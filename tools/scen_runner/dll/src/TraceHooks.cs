using System;
using System.Reflection;
using System.Collections.Generic;
using System.Collections;
using System.Runtime.CompilerServices;

// Logs are read-only. Cleanup entry points run only in the matching fix builds.
public static class ScenRunnerRuntimeTrace
{
    static readonly HashSet<string> ignoredPairs = new HashSet<string>();
    static readonly Dictionary<string,string> collisionSignatures = new Dictionary<string,string>();
    static readonly Dictionary<string,string> lifecycleSignatures = new Dictionary<string,string>();

    public static void RecordActorStateReturn(object result)
    {
        try {
            object state=Read(result,"Item2");
            if(state==null) state=result;
            object info=Read(state,"ActorInfo"), id=Read(Read(info,"Id"),"Value");
            if(id==null) return;
            object vehicle=Read(state,"VehicleState");
            IEnumerable contacts=Read(vehicle,"CollisionObjects") as IEnumerable;
            List<string> ids=new List<string>();
            if(contacts!=null) foreach(object contact in contacts) ids.Add(Convert.ToString(contact));
            ids.Sort();
            string signature="ok="+Read(result,"Item1")+";id="+id+";type="+Read(info,"ObjectType")+
                ";contacts="+String.Join(",",ids.ToArray());
            string slot="actor_state:"+id;
            lock(lifecycleSignatures) {
                string previous;
                if(lifecycleSignatures.TryGetValue(slot,out previous) && previous==signature) return;
                lifecycleSignatures[slot]=signature;
            }
            LogMessage("[SCEN_RUNTIME_TRACE] utc="+DateTime.UtcNow.ToString("o")+
                       " phase=contact_actor_state_return "+signature);
        } catch(Exception exc) {
            LogMessage("[SCEN_RUNTIME_TRACE] phase=contact_trace_error method=actor_state error="+exc.GetType().Name);
        }
    }

    static string ContactFields(object subject)
    {
        if(subject==null) return "null";
        List<string> fields=new List<string>();
        for(Type t=subject.GetType();t!=null;t=t.BaseType)
            foreach(FieldInfo f in t.GetFields(BindingFlags.Public|BindingFlags.NonPublic|BindingFlags.Instance|BindingFlags.DeclaredOnly)) {
                object value;
                try { value=f.GetValue(subject); } catch { continue; }
                IDictionary dictionary=value as IDictionary;
                if(dictionary!=null) {
                    List<string> entries=new List<string>();
                    foreach(DictionaryEntry entry in dictionary) {
                        object owner=Owner(entry.Key);
                        object first=Read(entry.Value,"Item1"), second=Read(entry.Value,"Item2");
                        entries.Add("key="+(InstanceId(entry.Key) ?? entry.Key)+",owner="+InstanceId(owner)+
                                    ",id="+Read(owner,"GetUniqueID")+",valueType="+
                                    (entry.Value==null?"null":entry.Value.GetType().Name)+
                                    ",counter1="+first+",counter2="+second);
                    }
                    entries.Sort();fields.Add(f.Name+"="+dictionary.Count+"["+String.Join("|",entries.ToArray())+"]");
                } else if(f.FieldType==typeof(bool)) fields.Add(f.Name+"="+value);
                else if(value is IList && f.FieldType.ToString().IndexOf("MeshCollision",StringComparison.Ordinal)>=0) {
                    List<string> items=new List<string>();
                    foreach(object item in (IList)value) items.Add(Convert.ToString(InstanceId(item)));
                    fields.Add(f.Name+"="+((IList)value).Count+"["+String.Join(",",items.ToArray())+"]");
                }
            }
        fields.Sort();return String.Join(";",fields.ToArray());
    }
    static void LifecycleRecord(string phase, object subject, object other)
    {
        object trigger=subject;
        Type triggerType=FindType("ObstacleBoxColliderTrigger");
        if(triggerType==null || !triggerType.IsInstanceOfType(trigger)) trigger=TypedField(subject,"ObstacleBoxColliderTrigger");
        string details=ContactFields(trigger);
        // Command objects expose request/response protobuf fields through reflection.
        // Read them without invoking any simulator control methods.
        List<string> messages=new List<string>();
        if(subject!=null && subject.GetType().Name.IndexOf("Command",StringComparison.Ordinal)>=0)
            for(Type t=subject.GetType();t!=null;t=t.BaseType)
                foreach(FieldInfo f in t.GetFields(BindingFlags.Public|BindingFlags.NonPublic|BindingFlags.Instance|BindingFlags.DeclaredOnly)) {
                    if(f.FieldType.FullName==null || f.FieldType.FullName.IndexOf("Protobuf",StringComparison.Ordinal)<0) continue;
                    object value=f.GetValue(subject);
                    if(value!=null) messages.Add(f.Name+"="+value.ToString());
                }
        string signature="subject="+Describe(subject)+";other="+Describe(other)+
            ";trigger="+InstanceId(trigger)+";fields="+details+";messages="+String.Join("|",messages.ToArray());
        bool repeatedQuery=phase.StartsWith("contact_query_",StringComparison.Ordinal);
        if(repeatedQuery) {
            // Query command instances can be allocated each RPC. Ignore their identity.
            signature="trigger="+InstanceId(trigger)+";fields="+details+";messages="+String.Join("|",messages.ToArray());
            lock(lifecycleSignatures) {
                string previous;
                if(lifecycleSignatures.TryGetValue(phase,out previous) && previous==signature) return;
                lifecycleSignatures[phase]=signature;
            }
        }
        LogMessage("[SCEN_RUNTIME_TRACE] utc="+DateTime.UtcNow.ToString("o")+" phase="+phase+
                   "\ncontact: "+signature);
    }

    // Compatibility entry point for the earlier, uninstalled vehicle-only candidate.
    public static void ClearRetiringVehicleContacts(object value) { ClearRetiringActorContacts(value); }

    public static void ClearRetiringActorContacts(object value)
    { ClearRetiringContacts(value, false); }

    // Only the static-contact build calls this entry point. Keep the older
    // vehicle/pedestrian build's admission policy intact.
    public static void ClearRetiringStaticContacts(object value)
    { ClearRetiringContacts(value, true); }

    static void ClearRetiringContacts(object value, bool staticsOnly)
    {
        try {
            object actor=Owner(value);
            // Some pool overloads receive a GameObject registered in the
            // project rather than a parent with a MoraiObjectBase component.
            if(actor==null && staticsOnly) actor=RegisteredOwner(value);
            if(actor==null) return;
            bool admitted;
            if(staticsOnly) {
                Type obstacle=FindType("ObstacleObject");
                admitted=obstacle!=null && obstacle.IsInstanceOfType(actor);
            } else {
                Type vehicle=FindType("MGVehicle"), pedestrian=FindType("PedestrianModel");
                admitted=(vehicle!=null && vehicle.IsInstanceOfType(actor)) ||
                         (pedestrian!=null && pedestrian.IsInstanceOfType(actor));
            }
            if(!admitted || Convert.ToInt32(Read(actor,"GetUniqueID"))<=0) return;
            Type triggerType=FindType("ObstacleBoxColliderTrigger"), unityObject=FindType("UnityEngine.Object");
            if(triggerType==null || unityObject==null) return;
            object actorInstance=InstanceId(actor);
            if(actorInstance==null) return;
            MethodInfo find=unityObject.GetMethod("FindObjectsOfType",new Type[]{typeof(Type),typeof(bool)});
            Array triggers=find!=null ? find.Invoke(null,new object[]{triggerType,true}) as Array :
                unityObject.GetMethod("FindObjectsOfType",new Type[]{typeof(Type)}).Invoke(null,new object[]{triggerType}) as Array;
            Type projectType=FindType("ProjectManager");
            object project=null;
            if(projectType!=null) {
                MethodInfo instance=projectType.GetMethod("get_instance",BindingFlags.Public|BindingFlags.Static);
                if(instance!=null) project=instance.Invoke(null,null);
            }
            Type gameObjectType=FindType("UnityEngine.GameObject");
            MethodInfo lookup=projectType==null || gameObjectType==null ? null : projectType.GetMethod("GetObjectBase",new Type[]{gameObjectType});
            int removed=0, mappings=0, counters=0;
            foreach(object trigger in triggers) {
                List<IDictionary> numericMaps=new List<IDictionary>(), contactCounters=new List<IDictionary>();
                HashSet<int> confirmedIds=new HashSet<int>();
                Type actorType=FindType("MoraiObjectBase");
                for(Type t=trigger.GetType();t!=null;t=t.BaseType)
                    foreach(FieldInfo field in t.GetFields(BindingFlags.Public|BindingFlags.NonPublic|BindingFlags.Instance|BindingFlags.DeclaredOnly)) {
                        if(!field.FieldType.IsGenericType) continue;
                        Type[] arguments=field.FieldType.GetGenericArguments();
                        if(arguments.Length!=2 || arguments[0]!=typeof(int)) continue;
                        IDictionary map=field.GetValue(trigger) as IDictionary;
                        if(map==null) continue;
                        if(actorType!=null && actorType.IsAssignableFrom(arguments[1])) {
                            numericMaps.Add(map);
                            foreach(DictionaryEntry entry in map)
                                if((int)entry.Key>0 && Object.Equals(InstanceId(entry.Value),actorInstance))
                                    confirmedIds.Add((int)entry.Key);
                        } else if(arguments[1].IsGenericType && arguments[1].GetGenericTypeDefinition().FullName=="System.ValueTuple`2") {
                            Type[] tuple=arguments[1].GetGenericArguments();
                            if(tuple[0]==typeof(int) && tuple[1]==typeof(int)) contactCounters.Add(map);
                        }
                    }
                // A numeric key is removed only after its owner mapping proves the
                // retiring Unity instance. Unrelated actors sharing an ID stay intact.
                foreach(int id in confirmedIds) {
                    foreach(IDictionary map in contactCounters) if(map.Contains(id)) { map.Remove(id);counters++; }
                    foreach(IDictionary map in numericMaps)
                        if(map.Contains(id) && Object.Equals(InstanceId(map[id]),actorInstance)) { map.Remove(id);mappings++; }
                }
                IDictionary dictionary=Read(trigger,"m_dicCollObject") as IDictionary;
                if(dictionary==null) continue;
                List<object> keys=new List<object>();
                foreach(object key in dictionary.Keys) keys.Add(key);
                foreach(object key in keys) {
                    object owner=Owner(key), ownerInstance=InstanceId(owner);
                    bool matches=ownerInstance!=null && Object.Equals(ownerInstance,actorInstance);
                    // The UDP collector resolves GameObject keys through the project registry.
                    if(!matches && project!=null && lookup!=null && gameObjectType.IsInstanceOfType(key)) {
                        object resolved=lookup.Invoke(project,new object[]{key});
                        object resolvedInstance=InstanceId(resolved);
                        matches=resolvedInstance!=null && Object.Equals(resolvedInstance,actorInstance);
                    }
                    if(matches) { dictionary.Remove(key); removed++; }
                }
            }
            string phase=staticsOnly ? "retired_static_contacts_removed" : "retired_actor_contacts_removed";
            LogMessage("[SCEN_RUNTIME_TRACE] utc="+DateTime.UtcNow.ToString("o")+" phase="+phase+" actor="+
                Read(actor,"GetUniqueID")+" instance="+actorInstance+" entries="+removed+" mappings="+mappings+" counters="+counters);
        } catch(Exception exc) {
            string phase=staticsOnly ? "retired_static_contacts_error" : "retired_actor_contacts_error";
            LogMessage("[SCEN_RUNTIME_TRACE] phase="+phase+" error="+exc.GetType().Name+":"+exc.Message);
        }
    }
    static void LogMessage(string message)
    {
        try {
            Type debug=FindType("UnityEngine.Debug");
            if(debug!=null) debug.GetMethod("Log",new Type[]{typeof(object)}).Invoke(null,new object[]{message});
        } catch { }
    }

    static object TypedField(object subject, string typeName)
    {
        Type wanted=FindType(typeName);
        if(subject==null || wanted==null) return null;
        for(Type t=subject.GetType();t!=null;t=t.BaseType)
            foreach(FieldInfo f in t.GetFields(BindingFlags.Public|BindingFlags.NonPublic|BindingFlags.Instance|BindingFlags.DeclaredOnly)) {
                if(!wanted.IsAssignableFrom(f.FieldType)) continue;
                object value=f.GetValue(subject); if(value!=null) return value;
            }
        return null;
    }
    static void CollisionRecord(string phase, object subject, object other)
    {
        object trigger=phase=="collision_source" ? TypedField(subject,"ObstacleBoxColliderTrigger") : subject;
        if(trigger==null) return;
        object owner=Owner(trigger), cachedOwner=TypedField(trigger,"MoraiObjectBase");
        if(phase!="collision_source") {
            object otherOwner=Owner(other);
            if(Convert.ToString(Read(owner,"GetUniqueID"))!="0" &&
               Convert.ToString(Read(otherOwner,"GetUniqueID"))!="0") return;
        }
        IDictionary dictionary=Read(trigger,"m_dicCollObject") as IDictionary;
        List<string> members=new List<string>();
        if(dictionary!=null) foreach(object key in dictionary.Keys) {
            object itemOwner=Owner(key);
            members.Add("key="+InstanceId(key)+",name="+Read(key,"name")+",owner="+InstanceId(itemOwner)+
                        ",id="+Read(itemOwner,"GetUniqueID"));
        }
        members.Sort();
        string signature="trigger="+InstanceId(trigger)+";hierarchyOwner="+InstanceId(owner)+
            ";cachedOwner="+InstanceId(cachedOwner)+";dict="+(dictionary==null?"null":RuntimeHelpers.GetHashCode(dictionary).ToString())+
            ";members="+String.Join("|",members.ToArray())+";other="+InstanceId(other);
        string slot=phase+":"+RuntimeHelpers.GetHashCode(subject);
        lock(collisionSignatures) {
            string previous;
            if(collisionSignatures.TryGetValue(slot,out previous) && previous==signature) return;
            collisionSignatures[slot]=signature;
        }
        object collider=TypedField(trigger,"UnityEngine.BoxCollider");
        string message="[SCEN_RUNTIME_TRACE] utc="+DateTime.UtcNow.ToString("o")+" phase="+phase+" "+signature+
            "\ncontactFields: "+ContactFields(trigger)+
            "\nsource: "+Describe(trigger)+"\ncachedOwner: "+Describe(cachedOwner)+
            "\nbounds: "+Read(collider,"bounds")+"\nother: "+Describe(other);
        Type debug=FindType("UnityEngine.Debug");
        if(debug!=null) debug.GetMethod("Log",new Type[]{typeof(object)}).Invoke(null,new object[]{message});
    }
    public static int SpawnBehavior(object model)
    {
        try {
            string name=Convert.ToString(Read(model,"name"));
            int behavior=Convert.ToInt32(Read(model,"pedestrianBehavior"));
            if(name.IndexOf("ped_pet1",StringComparison.OrdinalIgnoreCase)>=0 && behavior==7) {
                Record("pet_behavior_preserved",model,null);
                return behavior;
            }
        } catch { }
        return 0; // 그 외 모델은 기존 gRPC 초기화 인수를 유지한다.
    }
    public static bool SkipSelfCleanup(object subject, object other)
    {
        try {
            object a=InstanceId(Owner(subject)), b=InstanceId(Owner(other));
            if (a==null || b==null || !Object.Equals(a,b)) return false;
            string pair=InstanceId(subject)+":"+InstanceId(other)+":"+a;
            bool first;
            lock(ignoredPairs) first=ignoredPairs.Add(pair);
            if(first) Record("self_cleanup_ignored",subject,other);
            return true;
        } catch { return false; } // 소유자를 확인 못 하면 기존 처리를 유지한다.
    }
    static Type FindType(string name)
    {
        foreach (Assembly a in AppDomain.CurrentDomain.GetAssemblies()) {
            Type t = a.GetType(name, false); if (t != null) return t;
        }
        return null;
    }
    static object Read(object o, string name)
    {
        if (o == null) return null;
        try {
            Type t = o.GetType();
            PropertyInfo p = t.GetProperty(name, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
            if (p != null && p.GetIndexParameters().Length == 0) return p.GetValue(o, null);
            if(name=="GetUniqueID") {
                MethodInfo getter=t.GetMethod("get_GetUniqueID",BindingFlags.Public|BindingFlags.NonPublic|BindingFlags.Instance,
                                              null,Type.EmptyTypes,null);
                if(getter!=null) return getter.Invoke(o,null);
            }
            while (t != null) {
                FieldInfo f = t.GetField(name, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.DeclaredOnly);
                if (f != null) return f.GetValue(o); t = t.BaseType;
            }
        } catch { }
        return null;
    }
    static object InstanceId(object o)
    {
        try { return o.GetType().GetMethod("GetInstanceID", Type.EmptyTypes).Invoke(o, null); }
        catch { return null; }
    }
    static object Owner(object o)
    {
        if (o == null) return null;
        Type baseType = FindType("MoraiObjectBase");
        if (baseType == null) return null;
        if (baseType.IsInstanceOfType(o)) return o;
        object go = Read(o, "gameObject") ?? o;
        try {
            MethodInfo m = go.GetType().GetMethod("GetComponentInParent", new Type[] { typeof(Type), typeof(bool) });
            if (m != null) return m.Invoke(go, new object[] { baseType, true });
            m = go.GetType().GetMethod("GetComponentInParent", new Type[] { typeof(Type) });
            return m == null ? null : m.Invoke(go, new object[] { baseType });
        } catch { return null; }
    }
    static object RegisteredOwner(object o)
    {
        Type projectType=FindType("ProjectManager"), gameObjectType=FindType("UnityEngine.GameObject");
        if(o==null || projectType==null || gameObjectType==null || !gameObjectType.IsInstanceOfType(o)) return null;
        MethodInfo instance=projectType.GetMethod("get_instance",BindingFlags.Public|BindingFlags.Static);
        MethodInfo lookup=projectType.GetMethod("GetObjectBase",new Type[]{gameObjectType});
        if(instance==null || lookup==null) return null;
        object project=instance.Invoke(null,null);
        return project==null ? null : lookup.Invoke(project,new object[]{o});
    }
    static string Describe(object o)
    {
        if (o == null) return "null";
        object go = Read(o, "gameObject") ?? o;
        object owner = Owner(o);
        object tf = Read(o, "transform");
        return "type=" + o.GetType().Name + ";name=" + Read(o, "name") +
            ";instance=" + InstanceId(o) + ";tag=" + Read(go, "tag") +
            ";active=" + Read(go, "activeInHierarchy") + ";enabled=" + Read(o, "enabled") +
            ";position=" + Read(tf, "position") + ";ownerName=" + Read(owner, "name") +
            ";ownerType=" + (owner == null ? "null" : owner.GetType().Name) +
            ";ownerId=" + Read(owner, "GetUniqueID") + ";ownerInstance=" + InstanceId(owner) +
            ";request=" + Read(o, "requestMessage") + ";value=" + o.ToString();
    }
    public static void Record(string phase, object subject, object other)
    {
        try {
            if(phase.StartsWith("contact_",StringComparison.Ordinal)) {
                LifecycleRecord(phase,subject,other);return;
            }
            if(phase.StartsWith("collision_",StringComparison.Ordinal)) {
                CollisionRecord(phase,subject,other); return;
            }
            string a = Describe(subject), b = Describe(other);
            string combined = a + b;
            bool destroyCommand = phase.IndexOf("Destroy", StringComparison.Ordinal) >= 0 &&
                                  phase.EndsWith("ExecuteCore", StringComparison.Ordinal);
            // 일반 차량의 대량 로그를 피하고 보행자와 이 러너를 기록한다.
            if (!destroyCommand && combined.IndexOf("man6", StringComparison.OrdinalIgnoreCase) < 0 &&
                combined.IndexOf("Pedestrian", StringComparison.OrdinalIgnoreCase) < 0 &&
                combined.IndexOf("scen_runner", StringComparison.OrdinalIgnoreCase) < 0) return;
            object ownerA = Owner(subject), ownerB = Owner(other);
            object idA = InstanceId(ownerA), idB = InstanceId(ownerB);
            string same = idA == null || idB == null ? "unknown" : Object.Equals(idA, idB).ToString();
            string message = "[SCEN_RUNTIME_TRACE] utc=" + DateTime.UtcNow.ToString("o") +
                " phase=" + phase + " sameOwner=" + same + "\nsubject: " + a +
                "\nother: " + b + "\nstack: " + Environment.StackTrace;
            Type debug = FindType("UnityEngine.Debug");
            if (debug != null) debug.GetMethod("Log", new Type[] { typeof(object) }).Invoke(null, new object[] { message });
        } catch(Exception exc) {
            if(phase.StartsWith("contact_",StringComparison.Ordinal))
                LogMessage("[SCEN_RUNTIME_TRACE] phase=contact_trace_error site="+phase+" error="+exc.GetType().Name);
            /* 진단 실패가 게임의 기존 제어 흐름을 바꾸지 않도록 한다. */
        }
    }
}
