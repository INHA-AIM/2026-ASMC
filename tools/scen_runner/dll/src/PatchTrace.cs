using System;
using System.IO;
using System.Collections.Generic;
using Mono.Cecil;
using Mono.Cecil.Cil;

public static class ScenTraceBuilder
{
    static IEnumerable<TypeDefinition> Types(IEnumerable<TypeDefinition> types)
    {
        foreach (TypeDefinition t in types) {
            yield return t; foreach (TypeDefinition child in Types(t.NestedTypes)) yield return child;
        }
    }
    static void LongBranches(MethodDefinition m)
    {
        foreach (Instruction i in m.Body.Instructions) {
            if (i.OpCode.OperandType != OperandType.ShortInlineBrTarget) continue;
            string name = i.OpCode.Code.ToString();
            i.OpCode = (OpCode)typeof(OpCodes).GetField(name.Substring(0, name.Length-2)).GetValue(null);
        }
    }
    static Instruction Insert(MethodDefinition m, Instruction at, string phase, int arg, bool pair, MethodReference log)
    {
        LongBranches(m);
        ILProcessor il = m.Body.GetILProcessor();
        Instruction entry=il.Create(OpCodes.Ldstr,phase);
        il.InsertBefore(at, entry);
        if (arg < 0) il.InsertBefore(at, il.Create(OpCodes.Ldarg_0));
        else {
            ParameterDefinition p = m.Parameters[arg];
            il.InsertBefore(at, il.Create(OpCodes.Ldarg, p));
            if (p.ParameterType.IsValueType) il.InsertBefore(at, il.Create(OpCodes.Box, p.ParameterType));
        }
        if(pair) {
            il.InsertBefore(at,il.Create(OpCodes.Ldarg_1));
            if(m.Parameters[0].ParameterType.IsValueType)
                il.InsertBefore(at,il.Create(OpCodes.Box,m.Parameters[0].ParameterType));
        } else il.InsertBefore(at,il.Create(OpCodes.Ldnull));
        il.InsertBefore(at, il.Create(OpCodes.Call, log));
        m.Body.MaxStackSize = Math.Max(m.Body.MaxStackSize+4, 8);
        return entry;
    }
    static string Normalized(Instruction i, List<Instruction> instructions)
    {
        string code = i.OpCode.Code.ToString();
        if (i.OpCode.OperandType == OperandType.ShortInlineBrTarget) code=code.Substring(0,code.Length-2);
        object value=i.Operand;
        if (value is Instruction) value=instructions.IndexOf((Instruction)value);
        else if (value is Instruction[]) {
            string targets=""; foreach(Instruction target in (Instruction[])value) targets+=instructions.IndexOf(target)+",";
            value=targets;
        } else if (value is MethodReference) value=((MethodReference)value).FullName;
        else if (value is FieldReference) value=((FieldReference)value).FullName;
        else if (value is TypeReference) value=((TypeReference)value).FullName;
        else if (value is ParameterDefinition) value="param:"+((ParameterDefinition)value).Index;
        else if (value is VariableDefinition) value="local:"+((VariableDefinition)value).Index;
        return code+":"+value;
    }
    public static string Verify(string source, string output, bool expectFix, bool expectContacts)
    { return Verify(source,output,expectFix,expectContacts,false); }
    public static string Verify(string source, string output, bool expectFix, bool expectContacts, bool expectStatics)
    {
        using (AssemblyDefinition original=AssemblyDefinition.ReadAssembly(source))
        using (AssemblyDefinition patched=AssemblyDefinition.ReadAssembly(output)) {
            Dictionary<string,MethodDefinition> methods=new Dictionary<string,MethodDefinition>();
            foreach(TypeDefinition t in Types(original.MainModule.Types)) foreach(MethodDefinition m in t.Methods)
                if(m.HasBody) methods[m.FullName]=m;
            int verified=0, guards=0, petFixes=0, contactHooks=0, staticPoolHooks=0, staticReleaseHooks=0;
            foreach(TypeDefinition t in Types(patched.MainModule.Types)) foreach(MethodDefinition m in t.Methods) {
                if(!m.HasBody) continue;
                List<Instruction> clean=new List<Instruction>(m.Body.Instructions);
                bool changed=false;
                for(int n=clean.Count-1;n>=0;n--) {
                    MethodReference call=clean[n].Operand as MethodReference;
                    if(call==null||call.Name!="VehicleInitialized"||call.DeclaringType.Name!="ScenSpawnTrace")continue;
                    if(n<2||clean[n-2].OpCode.Code!=Code.Ldarg_0||clean[n-1].OpCode.Code!=Code.Ldloc||
                       m.DeclaringType.Name!="SpawnPointObject")throw new Exception("Unexpected vehicle initialization trace");
                    clean.RemoveRange(n-2,3);changed=true;n-=2;
                }
                for(int n=clean.Count-1;n>=0;n--) {
                    MethodReference call=clean[n].Operand as MethodReference;
                    if(call==null||call.Name!="SafetyVehicles"||call.DeclaringType.Name!="ScenSpawnTrace")continue;
                    int start=n-3;
                    if(start>=0 && clean[start].OpCode.Code==Code.Box)start--;
                    if(start<0||clean[start].OpCode.Code!=Code.Dup||clean[n-2].OpCode.Code!=Code.Ldarg_0||
                       clean[n-1].OpCode.Code!=Code.Ldstr||m.DeclaringType.Name!="SpawnPointObject")
                        throw new Exception("Unexpected native safety vehicle trace");
                    clean.RemoveRange(start,n-start+1);changed=true;n=start;
                }
                for(int n=clean.Count-1;n>=0;n--) {
                    MethodReference call=clean[n].Operand as MethodReference;
                    if(call==null || call.Name!="SafetyResult" || call.DeclaringType.Name!="ScenSpawnTrace")continue;
                    if(n<2 || clean[n-2].OpCode.Code!=Code.Dup || clean[n-1].OpCode.Code!=Code.Ldarg_0 ||
                       n+1>=clean.Count || clean[n+1].OpCode.Code!=Code.Ret || m.DeclaringType.Name!="SpawnPointObject")
                        throw new Exception("Unexpected spawn safety trace");
                    Instruction entry=clean[n-2],ret=clean[n+1];
                    foreach(Instruction branch in clean) {
                        if(branch.Operand==entry)branch.Operand=ret;
                        Instruction[] targets=branch.Operand as Instruction[];
                        if(targets!=null)for(int k=0;k<targets.Length;k++)if(targets[k]==entry)targets[k]=ret;
                    }
                    clean.RemoveRange(n-2,3);changed=true;n-=2;
                }
                for(int n=clean.Count-1;n>=0;n--) {
                    MethodReference call=clean[n].Operand as MethodReference;
                    if(call==null || call.Name!="RecordActorStateReturn" || call.DeclaringType.Name!="ScenRunnerRuntimeTrace") continue;
                    int start=n-1;
                    if(clean[start].OpCode.Code==Code.Box) start--;
                    if(start<0 || clean[start].OpCode.Code!=Code.Dup || m.DeclaringType.FullName!="CoreLib.Grpc.Utils" || m.Name!="GetActorState")
                        throw new Exception("Unexpected actor state return trace");
                    clean.RemoveRange(start,n-start+1);changed=true;n=start;
                }
                for(int n=clean.Count-1;n>=0;n--) {
                    MethodReference call=clean[n].Operand as MethodReference;
                    if(call==null || call.Name!="ClearRetiringStaticContacts" || call.DeclaringType.Name!="ScenRunnerRuntimeTrace") continue;
                    if(m.DeclaringType.Name=="MoraiObjectBase" && m.Name=="Release" && m.Parameters.Count==0 &&
                       n>0 && clean[n-1].OpCode.Code==Code.Ldarg_0) staticReleaseHooks++;
                    else {
                        ParameterDefinition parameter=n>0 ? clean[n-1].Operand as ParameterDefinition : null;
                        if(m.DeclaringType.Name!="ResourcesManager" || m.Name!="PushToPool" || parameter==null ||
                           clean[n-1].OpCode.Code!=Code.Ldarg ||
                           (parameter.ParameterType.Name!="GameObject" && parameter.ParameterType.Name!="MoraiObjectBase"))
                            throw new Exception("Unexpected static retirement hook");
                        staticPoolHooks++;
                    }
                    clean.RemoveRange(n-1,2);changed=true;n--;
                }
                for(int n=clean.Count-1;n>=0;n--) {
                    MethodReference call=clean[n].Operand as MethodReference;
                    if(call==null || call.Name!="ClearRetiringActorContacts" || call.DeclaringType.Name!="ScenRunnerRuntimeTrace") continue;
                    ParameterDefinition parameter=n>0 ? clean[n-1].Operand as ParameterDefinition : null;
                    if(m.DeclaringType.Name!="ResourcesManager" || m.Name!="PushToPool" || parameter==null ||
                       clean[n-1].OpCode.Code!=Code.Ldarg ||
                       (parameter.ParameterType.Name!="GameObject" && parameter.ParameterType.Name!="MoraiObjectBase"))
                        throw new Exception("Unexpected contact retirement hook");
                    clean.RemoveRange(n-1,2);contactHooks++;changed=true;n--;
                }
                for(int n=clean.Count-1;n>=0;n--) {
                    MethodReference call=clean[n].Operand as MethodReference;
                    if(call==null || call.Name!="SpawnBehavior" || call.DeclaringType.Name!="ScenRunnerRuntimeTrace") continue;
                    VariableDefinition variable=n>0 ? clean[n-1].Operand as VariableDefinition : null;
                    if(variable==null || variable.VariableType.Name!="PedestrianModel" || clean[n-1].OpCode.Code!=Code.Ldloc)
                        throw new Exception("Unexpected PET initialization patch");
                    clean[n-1]=Instruction.Create(OpCodes.Ldc_I4_0);
                    clean.RemoveAt(n);petFixes++;changed=true;n--;
                }
                for(int n=clean.Count-1;n>=0;n--) {
                    MethodReference call=clean[n].Operand as MethodReference;
                    if(call==null || call.Name!="SkipSelfCleanup" || call.DeclaringType.Name!="ScenRunnerRuntimeTrace") continue;
                    if(m.DeclaringType.Name!="ObstacleBoxColliderTrigger" || m.Name!="OnTriggerStay" || n<2 || n+2>=clean.Count ||
                       clean[n-2].OpCode.Code!=Code.Ldarg_0 || clean[n-1].OpCode.Code!=Code.Ldarg_1 ||
                       clean[n+1].OpCode.Code!=Code.Brfalse || clean[n+2].OpCode.Code!=Code.Ret)
                        throw new Exception("Unexpected self-contact guard");
                    Instruction target=clean[n+1].Operand as Instruction;
                    if(target==null || target.OpCode.Code!=Code.Ldstr || (string)target.Operand!="trigger_cleanup_3s")
                        throw new Exception("Guard does not preserve non-self cleanup path");
                    clean.RemoveRange(n-2,5); guards++; changed=true; n-=2;
                }
                for(int n=clean.Count-1;n>=0;n--) {
                    MethodReference call=clean[n].Operand as MethodReference;
                    if(call==null || call.Name!="Record" ||
                       (call.DeclaringType.Name!="ScenRunnerRuntimeTrace" && call.DeclaringType.Name!="ScenSpawnTrace")) continue;
                    int start=n-1;
                    while(start>=0 && clean[start].OpCode.Code!=Code.Ldstr) start--;
                    if(start<0 || clean[start].OpCode.Code!=Code.Ldstr) throw new Exception("Unexpected trace block");
                    string phase=(string)clean[start].Operand;
                    bool spawnPhase=phase=="poll"||phase=="attempt_enter"||phase=="attempt_exit"||
                        phase=="Setting_enter"||phase=="Setting_exit"||phase=="EnableSpawn_enter"||
                        phase=="EnableSpawn_exit"||phase=="Release_enter"||phase=="Release_exit";
                    if((call.DeclaringType.Name=="ScenSpawnTrace")!=spawnPhase)
                        throw new Exception("Trace helper/phase mismatch: "+phase);
                    clean.RemoveRange(start,n-start+1); changed=true; n=start;
                }
                if(!changed) continue;
                MethodDefinition old=methods[m.FullName];
                List<Instruction> before=new List<Instruction>(old.Body.Instructions);
                if(before.Count!=clean.Count) throw new Exception("Original instruction count changed: "+m.FullName);
                for(int n=0;n<before.Count;n++)
                    if(Normalized(before[n],before)!=Normalized(clean[n],clean))
                        throw new Exception("Original instruction or branch changed: "+m.FullName+" at "+n);
                if(old.Body.ExceptionHandlers.Count!=m.Body.ExceptionHandlers.Count) throw new Exception("Exception handlers changed");
                verified++;
            }
            if(verified<4) throw new Exception("Too few verified trace methods");
            if(guards!=(expectFix?1:0)) throw new Exception("Self-contact guard count mismatch");
            if(petFixes!=(expectFix?1:0)) throw new Exception("PET initialization count mismatch");
            if(expectContacts ? contactHooks==0 : contactHooks!=0) throw new Exception("Contact hook count mismatch");
            if(expectStatics ? (staticReleaseHooks!=1 || staticPoolHooks!=contactHooks || staticPoolHooks==0) :
                               (staticReleaseHooks!=0 || staticPoolHooks!=0)) throw new Exception("Static hook count mismatch");
            return "verified original instructions and branches in "+verified+" methods; self-contact guards="+guards+
                "; PET initialization fixes="+petFixes+"; retirement contact hooks="+contactHooks+
                "; static pool hooks="+staticPoolHooks+"; static release hooks="+staticReleaseHooks;
        }
    }
    public static string Build(string source, string helper, string output, bool fixSelf, bool fixContacts, bool lifecycleTrace)
    { return Build(source,helper,output,fixSelf,fixContacts,lifecycleTrace,false); }
    public static string Build(string source, string helper, string output, bool fixSelf, bool fixContacts, bool lifecycleTrace, bool spawnTrace)
    { return Build(source,helper,output,fixSelf,fixContacts,lifecycleTrace,spawnTrace,false); }
    public static string Build(string source, string helper, string output, bool fixSelf, bool fixContacts, bool lifecycleTrace, bool spawnTrace, bool fixStatics)
    {
        if(fixStatics && !fixContacts) throw new Exception("Static contact cleanup requires the actor contact fix");
        DefaultAssemblyResolver resolver = new DefaultAssemblyResolver();
        resolver.AddSearchDirectory(Path.GetDirectoryName(source));
        resolver.AddSearchDirectory(Path.GetDirectoryName(helper));
        ReaderParameters rp = new ReaderParameters { AssemblyResolver=resolver, InMemory=true };
        using (AssemblyDefinition asm = AssemblyDefinition.ReadAssembly(source, rp))
        using (AssemblyDefinition hooks = AssemblyDefinition.ReadAssembly(helper, rp)) {
            foreach (AssemblyNameReference r in asm.MainModule.AssemblyReferences)
                if (r.Name == "ScenRunner.RuntimeTrace") throw new Exception("이미 진단 로그가 삽입된 DLL입니다.");
            MethodDefinition record = null, skipSelf = null, spawnBehavior = null, stateReturn = null;
            foreach (TypeDefinition t in hooks.MainModule.Types)
                if(t.Name=="ScenRunnerRuntimeTrace")
                foreach (MethodDefinition m in t.Methods) {
                    if (m.Name == "Record") record = m;
                    if (m.Name == "SkipSelfCleanup") skipSelf = m;
                    if (m.Name == "SpawnBehavior") spawnBehavior = m;
                    if (m.Name == "RecordActorStateReturn") stateReturn = m;
                }
            MethodReference log = asm.MainModule.ImportReference(record);
            if(spawnTrace) {
                MethodDefinition spawnRecord=null,safetyRecord=null,vehicleRecord=null,birthRecord=null;
                foreach(TypeDefinition t in hooks.MainModule.Types)if(t.Name=="ScenSpawnTrace")
                    foreach(MethodDefinition m in t.Methods) {
                        if(m.Name=="Record")spawnRecord=m;
                        if(m.Name=="SafetyResult")safetyRecord=m;
                        if(m.Name=="SafetyVehicles")vehicleRecord=m;
                        if(m.Name=="VehicleInitialized")birthRecord=m;
                    }
                if(spawnRecord==null||safetyRecord==null||vehicleRecord==null||birthRecord==null)throw new Exception("Spawn trace helper missing");
                MethodReference spawnLog=asm.MainModule.ImportReference(spawnRecord);
                int attempts=0,safeties=0,polls=0;
                foreach(TypeDefinition t in Types(asm.MainModule.Types))foreach(MethodDefinition m in t.Methods) {
                    if(!m.HasBody||m.Body.Instructions.Count==0)continue;
                    if(t.FullName=="CoreLib.Grpc.Utils" && m.Name=="GetActorState") {
                        Insert(m,m.Body.Instructions[0],"poll",0,false,spawnLog);polls++;continue;
                    }
                    if(m.IsStatic)continue;
                    if(t.Name!="SpawnPointObject")continue;
                    bool safetyShape=m.ReturnType.FullName=="System.Boolean"&&m.Parameters.Count==5;
                    bool callsSpawnType=false,callsSafety=false,callsStart=false,callsForward=false;
                    foreach(Instruction i in m.Body.Instructions) {
                        MethodReference call=i.Operand as MethodReference;if(call==null)continue;
                        if(call.Name=="get_m_eNPCSpawnType")callsSpawnType=true;
                        if(call.Name=="get_m_fTTC")callsSafety=true;
                        if(call.Name=="get_m_startLinkInfo")callsStart=true;
                        if(call.Name=="GetVehiclesOnLinkForward")callsForward=true;
                    }
                    if(safetyShape&&callsStart&&callsForward) {
                        List<Instruction> returns=new List<Instruction>();
                        foreach(Instruction i in m.Body.Instructions)if(i.OpCode.Code==Code.Ret)returns.Add(i);
                        LongBranches(m);ILProcessor il=m.Body.GetILProcessor();
                        List<Instruction> queries=new List<Instruction>();
                        foreach(Instruction i in m.Body.Instructions) {
                            MethodReference call=i.Operand as MethodReference;
                            if(call!=null&&call.ReturnType.FullName!="System.Void"&&
                               (call.Name=="GetVehiclesOnLinkForward"||call.Name=="GetVehiclesOnLinkBack"||
                                call.Name=="CheckVehiclesOnLinkFrontRecursive"||call.Name=="CheckVehiclesOnLinkBackRecursive"))queries.Add(i);
                        }
                        foreach(Instruction query in queries) {
                            MethodReference call=(MethodReference)query.Operand;Instruction cursor=query;
                            Instruction dup=il.Create(OpCodes.Dup);il.InsertAfter(cursor,dup);cursor=dup;
                            if(call.ReturnType.IsValueType) {
                                Instruction box=il.Create(OpCodes.Box,call.ReturnType);il.InsertAfter(cursor,box);cursor=box;
                            }
                            Instruction owner=il.Create(OpCodes.Ldarg_0);il.InsertAfter(cursor,owner);cursor=owner;
                            Instruction label=il.Create(OpCodes.Ldstr,call.Name);il.InsertAfter(cursor,label);cursor=label;
                            il.InsertAfter(cursor,il.Create(OpCodes.Call,asm.MainModule.ImportReference(vehicleRecord)));
                        }
                        foreach(Instruction ret in returns) {
                            Instruction entry=il.Create(OpCodes.Dup);
                            foreach(Instruction branch in m.Body.Instructions) {
                                if(branch.Operand==ret)branch.Operand=entry;
                                Instruction[] targets=branch.Operand as Instruction[];
                                if(targets!=null)for(int k=0;k<targets.Length;k++)if(targets[k]==ret)targets[k]=entry;
                            }
                            il.InsertBefore(ret,entry);il.InsertBefore(ret,il.Create(OpCodes.Ldarg_0));
                            il.InsertBefore(ret,il.Create(OpCodes.Call,asm.MainModule.ImportReference(safetyRecord)));
                        }
                        m.Body.MaxStackSize+=4;safeties++;
                    } else if(callsSpawnType&&callsSafety&&m.ReturnType.FullName=="System.Void"&&m.Parameters.Count==0) {
                        Instruction init=null;VariableDefinition controller=null;
                        foreach(Instruction i in m.Body.Instructions) {
                            MethodReference call=i.Operand as MethodReference;
                            if(call!=null&&call.Name=="InitSVehicle")init=i;
                        }
                        foreach(VariableDefinition variable in m.Body.Variables)
                            if(variable.VariableType.Name=="MGVehicle") {
                                if(controller!=null)throw new Exception("Ambiguous spawned vehicle local");
                                controller=variable;
                            }
                        if(init==null||controller==null)throw new Exception("Native vehicle initialization shape changed");
                        ILProcessor il=m.Body.GetILProcessor();Instruction sourceArg=il.Create(OpCodes.Ldarg_0);
                        il.InsertAfter(init,sourceArg);Instruction vehicleArg=il.Create(OpCodes.Ldloc,controller);
                        il.InsertAfter(sourceArg,vehicleArg);
                        il.InsertAfter(vehicleArg,il.Create(OpCodes.Call,asm.MainModule.ImportReference(birthRecord)));
                        m.Body.MaxStackSize+=2;
                        Insert(m,m.Body.Instructions[0],"attempt_enter",-1,false,spawnLog);attempts++;
                    } else if(m.Name=="Setting"||m.Name=="EnableSpawn"||m.Name=="Release") {
                        // Entry snapshot records prior state; fallthrough snapshots record changes.
                        List<Instruction> returns=new List<Instruction>();
                        foreach(Instruction i in m.Body.Instructions)if(i.OpCode.Code==Code.Ret)returns.Add(i);
                        Insert(m,m.Body.Instructions[0],m.Name+"_enter",-1,false,spawnLog);
                        foreach(Instruction ret in returns)Insert(m,ret,m.Name+"_exit",-1,false,spawnLog);
                    }
                }
                if(attempts!=1||safeties!=1||polls!=2)throw new Exception("Unsupported spawn flow: attempts="+attempts+", safeties="+safeties+", polls="+polls);
            }
            if(fixContacts) {
                bool contactDictionary=false, ownerMapping=false, meshCounters=false;
                foreach(TypeDefinition t in Types(asm.MainModule.Types)) if(t.Name=="ObstacleBoxColliderTrigger")
                    foreach(FieldDefinition f in t.Fields) {
                        if(f.Name=="m_dicCollObject") contactDictionary=true;
                        GenericInstanceType dictionary=f.FieldType as GenericInstanceType;
                        if(dictionary==null || dictionary.ElementType.FullName!="System.Collections.Generic.Dictionary`2" ||
                           dictionary.GenericArguments.Count!=2 || dictionary.GenericArguments[0].FullName!="System.Int32") continue;
                        if(dictionary.GenericArguments[1].Name=="MoraiObjectBase") ownerMapping=true;
                        GenericInstanceType pair=dictionary.GenericArguments[1] as GenericInstanceType;
                        if(pair!=null && pair.ElementType.FullName=="System.ValueTuple`2" && pair.GenericArguments.Count==2 &&
                           pair.GenericArguments[0].FullName=="System.Int32" && pair.GenericArguments[1].FullName=="System.Int32") meshCounters=true;
                    }
                if(!contactDictionary || !ownerMapping || !meshCounters)
                    throw new Exception("Unsupported collision storage layout. No simulator files changed.");
            }
            MethodDefinition staticClear=null;
            if(fixStatics) {
                TypeDefinition obstacle=null, actorBase=null;
                foreach(TypeDefinition t in Types(asm.MainModule.Types)) {
                    if(t.Name=="ObstacleObject") obstacle=t;
                    if(t.Name=="MoraiObjectBase") actorBase=t;
                }
                if(obstacle==null || obstacle.BaseType==null || obstacle.BaseType.Name!="MoraiObjectBase" || actorBase==null)
                    throw new Exception("Unsupported static actor type. No simulator files changed.");
                MethodDefinition release=null;
                foreach(MethodDefinition m in actorBase.Methods)
                    if(m.Name=="Release" && !m.IsStatic && m.Parameters.Count==0 && m.HasBody) release=m;
                bool releasesTrigger=false, retiresId=false;
                if(release!=null) foreach(Instruction i in release.Body.Instructions) {
                    MethodReference called=i.Operand as MethodReference;
                    if(called!=null && called.DeclaringType.Name=="ObstacleBoxColliderTrigger" && called.Name=="Release") releasesTrigger=true;
                    if(called!=null && called.DeclaringType.Name=="MoraiObjectBase" && called.Name=="set_m_ActivatingSensorUniqueId") retiresId=true;
                }
                if(!releasesTrigger || !retiresId) throw new Exception("Unsupported static release flow. No simulator files changed.");
                foreach(TypeDefinition t in hooks.MainModule.Types) foreach(MethodDefinition m in t.Methods)
                    if(t.Name=="ScenRunnerRuntimeTrace" && m.Name=="ClearRetiringStaticContacts") staticClear=m;
                if(staticClear==null) throw new Exception("Static cleanup helper missing");
            }
            if(fixSelf) {
                MethodDefinition factory=null;
                foreach(TypeDefinition t in Types(asm.MainModule.Types)) if(t.Name=="PedestrianSpawnCommand")
                    foreach(MethodDefinition command in t.Methods) if(command.Name=="ExecuteCore")
                        foreach(Instruction i in command.Body.Instructions) {
                            MethodReference reference=i.Operand as MethodReference;
                            if(reference==null || reference.DeclaringType.Scope!=asm.MainModule) continue;
                            MethodDefinition candidate=reference.Resolve();
                            if(candidate==null || !candidate.HasBody) continue;
                            foreach(Instruction inside in candidate.Body.Instructions) {
                                MethodReference init=inside.Operand as MethodReference;
                                if(init!=null && init.DeclaringType.Name=="PedestrianModel" && init.Name=="InitData") factory=candidate;
                            }
                        }
                if(factory==null) throw new Exception("Pedestrian spawn factory not found");
                Instruction behavior=null;
                for(int n=6;n<factory.Body.Instructions.Count;n++) {
                    MethodReference init=factory.Body.Instructions[n].Operand as MethodReference;
                    if(init==null || init.DeclaringType.Name!="PedestrianModel" || init.Name!="InitData") continue;
                    if(init.Parameters.Count!=22 || factory.Body.Instructions[n-6].OpCode.Code!=Code.Ldc_I4_0 ||
                       factory.Body.Instructions[n-5].OpCode.Code!=Code.Ldnull || factory.Body.Instructions[n-4].OpCode.Code!=Code.Ldnull)
                        throw new Exception("Spawn InitData argument shape differs");
                    behavior=factory.Body.Instructions[n-6];
                }
                VariableDefinition model=null;
                foreach(VariableDefinition v in factory.Body.Variables) if(v.VariableType.Name=="PedestrianModel") {
                    if(model!=null) throw new Exception("Ambiguous model local"); model=v;
                }
                if(behavior==null || model==null) throw new Exception("PET argument/local not found");
                behavior.OpCode=OpCodes.Ldloc;behavior.Operand=model;
                factory.Body.GetILProcessor().InsertAfter(behavior,Instruction.Create(OpCodes.Call,asm.MainModule.ImportReference(spawnBehavior)));
                LongBranches(factory);
            }
            int count = 0, triggerCount = 0, collisionSourceCount=0, retirementCount=0, staticReleaseCount=0;
            int lifecycleSites=0;
            if(lifecycleTrace) {
                foreach(TypeDefinition t in Types(asm.MainModule.Types)) foreach(MethodDefinition m in t.Methods) {
                    if(!m.HasBody || m.Body.Instructions.Count==0) continue;
                    if(t.FullName=="CoreLib.Grpc.Utils" && m.Name=="GetActorState") {
                        if(stateReturn==null || m.ReturnType.FullName=="System.Void") throw new Exception("Actor state return trace shape differs");
                        List<Instruction> stateReturns=new List<Instruction>();
                        foreach(Instruction i in m.Body.Instructions) if(i.OpCode.Code==Code.Ret) stateReturns.Add(i);
                        foreach(Instruction ret in stateReturns) {
                            ILProcessor il=m.Body.GetILProcessor();
                            il.InsertBefore(ret,il.Create(OpCodes.Dup));
                            if(m.ReturnType.IsValueType) il.InsertBefore(ret,il.Create(OpCodes.Box,m.ReturnType));
                            il.InsertBefore(ret,il.Create(OpCodes.Call,asm.MainModule.ImportReference(stateReturn)));
                            lifecycleSites++;
                        }
                        m.Body.MaxStackSize=Math.Max(m.Body.MaxStackSize+2,8);
                        continue;
                    }
                    if(m.IsStatic) continue;
                    string phase=null;
                    bool pair=false;
                    if(t.Name=="PedestrianModel" && m.Name=="InitData" && m.Parameters.Count==22)
                        phase="contact_init";
                    else if(t.Name=="ObstacleBoxColliderTrigger" &&
                            (m.Name=="EnableAccurateCollisionTrigger" || m.Name=="OnMeshCollisionEnterEvent" ||
                             m.Name=="OnMeshCollisionExitEvent" || m.Name=="Release" || m.Name=="SetObject")) {
                        phase="contact_"+m.Name;
                        pair=m.Parameters.Count==1;
                    } else if(t.Name=="MeshCollisionEventNotifier" &&
                              (m.Name=="SetCollisionEventReceiver" || m.Name=="OnCollisionEnter" || m.Name=="OnCollisionExit")) {
                        phase="contact_notifier_"+m.Name;
                        pair=m.Parameters.Count==1;
                    } else if((t.Name=="GetAllActorsStateCommand" || t.Name=="GetActorStateCommand") && m.Name=="ExecuteCore")
                        phase="contact_query_"+t.Name;
                    if(phase==null) continue;
                    List<Instruction> returns=new List<Instruction>();
                    foreach(Instruction i in m.Body.Instructions) if(i.OpCode.Code==Code.Ret) returns.Add(i);
                    Insert(m,m.Body.Instructions[0],phase+"_enter",-1,pair,log);lifecycleSites++;
                    // Keep existing branch targets intact. Early branches directly to ret
                    // can bypass this fall-through marker; entry is always recorded.
                    foreach(Instruction ret in returns) {
                        Insert(m,ret,phase+"_return_fallthrough",-1,pair,log);lifecycleSites++;
                    }
                }
                if(lifecycleSites<10) throw new Exception("Lifecycle trace sites missing");
            }
            foreach (TypeDefinition t in Types(asm.MainModule.Types)) foreach (MethodDefinition m in t.Methods) {
                if (!m.HasBody || m.IsStatic || m.Body.Instructions.Count == 0) continue;
                if (t.Name == "ObstacleBoxColliderTrigger" && m.Name == "OnTriggerStay") {
                    Instruction pivot = null;
                    foreach (Instruction i in m.Body.Instructions)
                        if (i.OpCode.Code == Code.Ldc_R4 && (float)i.Operand == 3.0f &&
                            i.Next.OpCode.FlowControl == FlowControl.Cond_Branch) pivot = i.Next.Next;
                    if (pivot == null || m.Parameters.Count != 1) throw new Exception("3초 제거 분기의 형태가 예상과 다릅니다.");
                    Instruction cleanup=Insert(m, pivot, "trigger_cleanup_3s", -1, true, log);
                    if(fixSelf) {
                        ILProcessor il=m.Body.GetILProcessor();
                        il.InsertBefore(cleanup,il.Create(OpCodes.Ldarg_0));
                        il.InsertBefore(cleanup,il.Create(OpCodes.Ldarg_1));
                        il.InsertBefore(cleanup,il.Create(OpCodes.Call,asm.MainModule.ImportReference(skipSelf)));
                        il.InsertBefore(cleanup,il.Create(OpCodes.Brfalse,cleanup));
                        il.InsertBefore(cleanup,il.Create(OpCodes.Ret));
                    }
                    count++; triggerCount++;
                } else if(t.Name=="ObstacleBoxColliderTrigger" &&
                          (m.Name=="OnTriggerEnter" || m.Name=="OnTriggerExit") && m.Parameters.Count==1) {
                    Insert(m,m.Body.Instructions[0],m.Name=="OnTriggerEnter"?"collision_enter":"collision_exit",-1,true,log);
                    count++;
                } else if(t.Name=="CollisionData" && t.Namespace=="") {
                    bool readsDictionary=false, readsMode=false, unknownMarker=false;
                    foreach(Instruction i in m.Body.Instructions) {
                        FieldReference field=i.Operand as FieldReference;
                        if(field!=null && field.Name=="m_dicCollObject") readsDictionary=true;
                        if(field!=null && field.Name=="m_vehicleMode") readsMode=true;
                        if((i.OpCode.Code==Code.Ldc_I4 || i.OpCode.Code==Code.Ldc_I4_S) && Convert.ToInt32(i.Operand)==-100)
                            unknownMarker=true;
                    }
                    // Match the actual collector in this build, rejecting fake obfuscated copies.
                    if(readsDictionary && readsMode && unknownMarker) {
                        Insert(m,m.Body.Instructions[0],"collision_source",-1,false,log);
                        count++;collisionSourceCount++;
                    }
                } else if ((t.Name == "ResourcesManager" && m.Name == "PushToPool") ||
                           (t.Name == "ObjectManager" && m.Name == "DestroyPathManager")) {
                    int arg = -1;
                    for (int n=0; n<m.Parameters.Count; n++) {
                        string type = m.Parameters[n].ParameterType.FullName;
                        if (type == "UnityEngine.GameObject" || type == "MoraiObjectBase") arg = n;
                    }
                    if (arg >= 0) {
                        Instruction entry=Insert(m, m.Body.Instructions[0], t.Name+"."+m.Name, arg, false, log); count++;
                        if(fixContacts && t.Name=="ResourcesManager") {
                            MethodDefinition clear=null;
                            foreach(TypeDefinition ht in Types(hooks.MainModule.Types))
                                foreach(MethodDefinition hm in ht.Methods) if(hm.Name=="ClearRetiringActorContacts") clear=hm;
                            if(clear==null) throw new Exception("Contact helper missing");
                            ILProcessor il=m.Body.GetILProcessor();
                            il.InsertBefore(entry,il.Create(OpCodes.Ldarg,m.Parameters[arg]));
                            il.InsertBefore(entry,il.Create(OpCodes.Call,asm.MainModule.ImportReference(clear)));
                            if(fixStatics) {
                                il.InsertBefore(entry,il.Create(OpCodes.Ldarg,m.Parameters[arg]));
                                il.InsertBefore(entry,il.Create(OpCodes.Call,asm.MainModule.ImportReference(staticClear)));
                            }
                            retirementCount++;
                        }
                    }
                } else if ((t.Name == "MoraiObjectBase" && m.Name == "Release") ||
                           (t.Name.IndexOf("Destroy", StringComparison.Ordinal) >= 0 &&
                            t.Name.IndexOf("Command", StringComparison.Ordinal) >= 0 && m.Name == "ExecuteCore")) {
                    Instruction entry=Insert(m, m.Body.Instructions[0], t.Name+"."+m.Name, -1, false, log); count++;
                    if(fixStatics && t.Name=="MoraiObjectBase" && m.Name=="Release") {
                        ILProcessor il=m.Body.GetILProcessor();
                        il.InsertBefore(entry,il.Create(OpCodes.Ldarg_0));
                        il.InsertBefore(entry,il.Create(OpCodes.Call,asm.MainModule.ImportReference(staticClear)));
                        staticReleaseCount++;
                    }
                }
            }
            if (triggerCount != 1 || count < 4 || collisionSourceCount!=1) throw new Exception("필수 로그 지점 확인 실패");
            if(fixContacts && retirementCount==0) throw new Exception("Pool retirement hook missing");
            if(fixStatics && staticReleaseCount!=1) throw new Exception("Static release hook missing");
            asm.Write(output);
            return "trace sites="+count+", trigger_cleanup_3s="+triggerCount+", collision_source="+collisionSourceCount+", lifecycle sites="+lifecycleSites;
        }
    }
}
