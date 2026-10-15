using System;
using System.Collections.Generic;
using System.IO;
using Mono.Cecil;
using Mono.Cecil.Cil;

// Preserve the installed fixes. The only added behavior is returning before
// native Transform.Find/DestroyImmediate when the requested path name is empty.
public static class ScenPathCleanupBuilder
{
    static IEnumerable<TypeDefinition> Types(IEnumerable<TypeDefinition> types)
    {
        foreach (TypeDefinition type in types) {
            yield return type;
            foreach (TypeDefinition child in Types(type.NestedTypes)) yield return child;
        }
    }
    static MethodDefinition Target(AssemblyDefinition assembly)
    {
        MethodDefinition found = null;
        foreach (TypeDefinition type in Types(assembly.MainModule.Types))
            foreach (MethodDefinition method in type.Methods)
                if (type.FullName == "SWS.WaypointManager" && method.Name == "DeletePathManager" &&
                    method.IsStatic && method.HasBody && method.ReturnType.FullName == "System.Void" &&
                    method.Parameters.Count == 1 && method.Parameters[0].ParameterType.FullName == "System.String") {
                    if (found != null) throw new Exception("Ambiguous native path cleanup method.");
                    found = method;
                }
        if (found == null) throw new Exception("Supported static DeletePathManager(string) not found.");
        return found;
    }
    public static string Build(string source, string output)
    {
        DefaultAssemblyResolver resolver = new DefaultAssemblyResolver();
        resolver.AddSearchDirectory(Path.GetDirectoryName(source));
        using (AssemblyDefinition assembly = AssemblyDefinition.ReadAssembly(source,
            new ReaderParameters { AssemblyResolver = resolver, InMemory = true })) {
            foreach (AssemblyNameReference reference in assembly.MainModule.AssemblyReferences)
                if (reference.Name == "ScenRunner.ActivationTrace")
                    throw new Exception("Remove the activation diagnostic before using it as the fix base.");
            MethodDefinition method = Target(assembly);
            int finds = 0, destroys = 0;
            foreach (Instruction instruction in method.Body.Instructions) {
                MethodReference call = instruction.Operand as MethodReference;
                if (call == null) continue;
                if (call.DeclaringType.FullName == "System.String" && call.Name == "IsNullOrEmpty")
                    throw new Exception("Path cleanup already has an empty-name guard. No duplicate patch created.");
                if (call.DeclaringType.FullName == "UnityEngine.Transform" && call.Name == "Find") finds++;
                if (call.DeclaringType.FullName == "UnityEngine.Object" && call.Name == "DestroyImmediate") destroys++;
            }
            if (finds != 1 || destroys != 1) throw new Exception("Unsupported native path cleanup structure.");
            ModuleDefinition module = assembly.MainModule;
            MethodReference empty = new MethodReference("IsNullOrEmpty", module.TypeSystem.Boolean, module.TypeSystem.String);
            empty.Parameters.Add(new ParameterDefinition(module.TypeSystem.String));
            Instruction first = method.Body.Instructions[0];
            ILProcessor il = method.Body.GetILProcessor();
            il.InsertBefore(first, il.Create(OpCodes.Ldarg_0));
            il.InsertBefore(first, il.Create(OpCodes.Call, empty));
            il.InsertBefore(first, il.Create(OpCodes.Brfalse, first));
            il.InsertBefore(first, il.Create(OpCodes.Ret));
            method.Body.MaxStackSize = Math.Max(method.Body.MaxStackSize, 1);
            assembly.Write(output);
            return "Added one empty-name guard to " + method.FullName + "; no new helper or logging dependency.";
        }
    }
    static string InstructionText(Instruction instruction, IList<Instruction> body)
    {
        object operand = instruction.Operand;
        if (operand is Instruction) operand = body.IndexOf((Instruction)operand);
        else if (operand is Instruction[]) {
            List<string> targets = new List<string>();
            foreach (Instruction target in (Instruction[])operand) targets.Add(body.IndexOf(target).ToString());
            operand = String.Join(",", targets.ToArray());
        } else if (operand is MethodReference) operand = ((MethodReference)operand).FullName;
        else if (operand is FieldReference) operand = ((FieldReference)operand).FullName;
        else if (operand is TypeReference) operand = ((TypeReference)operand).FullName;
        else if (operand is ParameterDefinition) operand = "parameter:" + ((ParameterDefinition)operand).Index;
        else if (operand is VariableDefinition) operand = "local:" + ((VariableDefinition)operand).Index;
        return instruction.OpCode.Code + ":" + operand;
    }
    static int Edge(Instruction instruction, IList<Instruction> body)
    { return instruction == null ? -1 : body.IndexOf(instruction); }
    static Dictionary<string, MethodDefinition> Bodies(AssemblyDefinition assembly)
    {
        Dictionary<string, MethodDefinition> result = new Dictionary<string, MethodDefinition>();
        foreach (TypeDefinition type in Types(assembly.MainModule.Types))
            foreach (MethodDefinition method in type.Methods) if (method.HasBody) result.Add(method.FullName, method);
        return result;
    }
    public static string Verify(string source, string output)
    {
        using (AssemblyDefinition original = AssemblyDefinition.ReadAssembly(source))
        using (AssemblyDefinition patched = AssemblyDefinition.ReadAssembly(output)) {
            string target = Target(original).FullName;
            Dictionary<string, MethodDefinition> before = Bodies(original), after = Bodies(patched);
            if (before.Count != after.Count) throw new Exception("Native method count changed.");
            int guarded = 0;
            foreach (KeyValuePair<string, MethodDefinition> pair in before) {
                MethodDefinition old = pair.Value, method;
                if (!after.TryGetValue(pair.Key, out method)) throw new Exception("Native method missing.");
                List<Instruction> clean = new List<Instruction>(method.Body.Instructions);
                if (pair.Key == target) {
                    MethodReference call = clean.Count > 4 ? clean[1].Operand as MethodReference : null;
                    if (clean.Count != old.Body.Instructions.Count + 4 || clean[0].OpCode.Code != Code.Ldarg_0 ||
                        clean[1].OpCode.Code != Code.Call || call == null ||
                        call.FullName != "System.Boolean System.String::IsNullOrEmpty(System.String)" ||
                        call.HasThis || clean[2].OpCode.Code != Code.Brfalse || clean[2].Operand != clean[4] ||
                        clean[3].OpCode.Code != Code.Ret) throw new Exception("Empty-name guard shape mismatch.");
                    clean.RemoveRange(0, 4); guarded++;
                }
                if (old.Attributes != method.Attributes || old.ImplAttributes != method.ImplAttributes ||
                    old.Body.InitLocals != method.Body.InitLocals || old.Body.Variables.Count != method.Body.Variables.Count ||
                    old.Body.ExceptionHandlers.Count != method.Body.ExceptionHandlers.Count ||
                    old.Body.Instructions.Count != clean.Count) throw new Exception("Native body structure changed: " + pair.Key);
                for (int i = 0; i < old.Body.Variables.Count; i++)
                    if (old.Body.Variables[i].VariableType.FullName != method.Body.Variables[i].VariableType.FullName)
                        throw new Exception("Native local type changed.");
                for (int i = 0; i < clean.Count; i++)
                    if (InstructionText(old.Body.Instructions[i], old.Body.Instructions) != InstructionText(clean[i], clean))
                        throw new Exception("Native instruction/branch changed: " + pair.Key + " at " + i);
                for (int i = 0; i < old.Body.ExceptionHandlers.Count; i++) {
                    ExceptionHandler x = old.Body.ExceptionHandlers[i], y = method.Body.ExceptionHandlers[i];
                    if (x.HandlerType != y.HandlerType || (x.CatchType == null ? null : x.CatchType.FullName) !=
                        (y.CatchType == null ? null : y.CatchType.FullName) ||
                        Edge(x.TryStart, old.Body.Instructions) != Edge(y.TryStart, clean) ||
                        Edge(x.TryEnd, old.Body.Instructions) != Edge(y.TryEnd, clean) ||
                        Edge(x.HandlerStart, old.Body.Instructions) != Edge(y.HandlerStart, clean) ||
                        Edge(x.HandlerEnd, old.Body.Instructions) != Edge(y.HandlerEnd, clean) ||
                        Edge(x.FilterStart, old.Body.Instructions) != Edge(y.FilterStart, clean))
                        throw new Exception("Native exception region changed: " + pair.Key);
                }
            }
            if (guarded != 1) throw new Exception("Expected exactly one added path guard.");
            return "PASS: all " + before.Count + " native method bodies/branches/exception regions and prior fixes preserved; one four-instruction empty-name guard.";
        }
    }
}
