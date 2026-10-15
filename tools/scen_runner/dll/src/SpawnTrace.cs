using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;

// Read-only native spawn diagnostics. No simulator setter/control calls.
public static class ScenSpawnTrace
{
    static DateTime nextPoll=DateTime.MinValue;
    static readonly Dictionary<string,string> signatures=new Dictionary<string,string>();
    static readonly Dictionary<string,DateTime> nextReport=new Dictionary<string,DateTime>();
    static readonly Dictionary<string,string> safety=new Dictionary<string,string>();
    static readonly Dictionary<string,int> attempts=new Dictionary<string,int>();
    static readonly Dictionary<string,int> checks=new Dictionary<string,int>();
    static readonly Dictionary<string,string> lastStage=new Dictionary<string,string>();
    static Type Find(string name) {
        foreach(Assembly a in AppDomain.CurrentDomain.GetAssemblies()) {
            Type t=a.GetType(name,false);if(t!=null)return t;
        }
        return null;
    }
    static object Read(object value,string name) {
        if(value==null)return null;
        for(Type t=value.GetType();t!=null;t=t.BaseType) {
            var flags=BindingFlags.Public|BindingFlags.NonPublic|BindingFlags.Instance|BindingFlags.DeclaredOnly;
            PropertyInfo p=t.GetProperty(name,flags);
            if(p!=null && p.GetIndexParameters().Length==0)return p.GetValue(value,null);
            MethodInfo m=t.GetMethod("get_"+name,flags,null,Type.EmptyTypes,null);
            if(m!=null)return m.Invoke(value,null);
            if(name=="GetIsMakeNotFind"||name=="GetObjectType") {
                m=t.GetMethod(name,flags,null,Type.EmptyTypes,null);
                if(m!=null)return m.Invoke(value,null);
            }
            FieldInfo f=t.GetField(name,flags);if(f!=null)return f.GetValue(value);
        }
        return null;
    }
    static string Text(object value) { return value==null?"unavailable":Convert.ToString(value,System.Globalization.CultureInfo.InvariantCulture); }
    static string Identity(object point) {
        MethodInfo m=point.GetType().GetMethod("GetInstanceID",Type.EmptyTypes);
        return Text(Read(point,"GetUniqueID"))+":"+(m==null?"unavailable":Text(m.Invoke(point,null)));
    }
    static void Emit(string key,string phase,string details,bool force) {
        string old;DateTime due;
        if(!force && signatures.TryGetValue(key,out old) && old==details &&
           nextReport.TryGetValue(key,out due) && DateTime.UtcNow<due)return;
        signatures[key]=details;nextReport[key]=DateTime.UtcNow.AddSeconds(5);
        Type debug=Find("UnityEngine.Debug");
        if(debug!=null)debug.GetMethod("Log",new Type[]{typeof(object)}).Invoke(null,new object[]{
            "[SCEN_SPAWN_TRACE] utc="+DateTime.UtcNow.ToString("o")+" phase="+phase+" "+details});
    }
    static void Snapshot(object point,string phase,bool force) {
        string key=Identity(point), verdict, stage;int count=0,checkCount=0;
        safety.TryGetValue(key,out verdict);attempts.TryGetValue(key,out count);checks.TryGetValue(key,out checkCount);
        lastStage.TryGetValue(key,out stage);
        object transform=Read(point,"transform"), go=Read(point,"gameObject");
        string details="id_instance="+key+" active="+Text(Read(go,"activeInHierarchy"))+
            " spawning="+Text(Read(point,"m_isSpawning"))+
            " count="+Text(Read(point,"m_spawnedVehicleCount"))+" maximum="+Text(Read(point,"m_maximumSpawnVehicle"))+
            " start="+Text(Read(Read(point,"m_startLinkInfo"),"linkIdx"))+
            " start_index="+Text(Read(Read(point,"m_startLinkInfo"),"pointIdx"))+
            " position_unity="+Text(Read(transform,"position"))+
            " model_id="+Text(Read(point,"m_spawnVehicleTypeID"))+
            " target_min="+Text(Read(point,"m_fMinDesiredVelocity_Custom"))+
            " target_max="+Text(Read(point,"m_fMaxDesiredVelocity_Custom"))+
            " wait_s="+Text(Read(point,"m_waitSpawnTime"))+
            " spawn_type="+Text(Read(point,"m_eNPCSpawnType"))+
            " safety_m="+Text(Read(point,"m_fSafetyDist"))+" ttc_s="+Text(Read(point,"m_fTTC"))+
            " attempts="+count+" safety_checks="+checkCount+" last_attempt="+(stage??"none")+
            " last_unsafe="+(verdict??"not_checked");
        // Counters change every frame. Throttle periodic snapshots independently.
        if(phase=="snapshot") {
            if(Object.Equals(Read(point,"m_isSpawning"),false)) {
                string old;if(signatures.TryGetValue(phase+":"+key,out old)&&old==details)return;
            }
            DateTime due;if(nextReport.TryGetValue("poll:"+key,out due) && DateTime.UtcNow<due)return;
            nextReport["poll:"+key]=DateTime.UtcNow.AddSeconds(5);
        }
        Emit(phase+":"+key,phase,details,force);
    }
    public static void SafetyResult(bool unsafeResult,object point) {
        try {
            string key=Identity(point);int n;checks.TryGetValue(key,out n);checks[key]=n+1;
            string before;string value=unsafeResult?"true":"false";
            bool changed=!safety.TryGetValue(key,out before)||before!=value;
            safety[key]=value;
            if(changed)Snapshot(point,"safety_result",true);
        } catch(Exception e) { Error(e); }
    }
    public static void VehicleInitialized(object point,object vehicle) {
        try {
            Emit("birth:"+Identity(point),"vehicle_initialized",
                 "id_instance="+Identity(point)+" vehicle_id_instance="+Identity(vehicle)+
                 " vehicle_type="+vehicle.GetType().Name+" object_type="+Text(Read(vehicle,"GetObjectType"))+
                 " excluded_from_all="+Text(Read(vehicle,"GetIsMakeNotFind"))+
                 " active="+Text(Read(Read(vehicle,"gameObject"),"activeInHierarchy"))+
                 " position_unity="+Text(Read(Read(vehicle,"transform"),"position")),true);
        }catch(Exception e){Error(e);}
    }
    public static void SafetyVehicles(object result,object point,string query) {
        try {
            string key=Identity(point)+":"+query;
            IEnumerable list=result as IEnumerable;List<string> items=new List<string>();
            if(list!=null)foreach(object item in list) {
                List<string> fields=new List<string>();
                if(item!=null)foreach(FieldInfo f in item.GetType().GetFields(BindingFlags.Public|BindingFlags.NonPublic|BindingFlags.Instance)) {
                    object value=f.GetValue(item);
                    if(f.FieldType.IsPrimitive||f.FieldType.IsEnum||value is string||f.FieldType.Name=="Vector3")
                        fields.Add(f.Name+"="+Text(value));
                    else if(value!=null) {
                        object id=Read(value,"GetUniqueID");
                        if(id!=null)fields.Add(f.Name+".actor_id="+Text(id));
                    }
                }
                items.Add(String.Join(",",fields.ToArray()));
                if(items.Count>=12){items.Add("truncated");break;}
            }
            string details="id_instance="+Identity(point)+" query="+query+" entries="+
                (list==null?Text(result):String.Join("|",items.ToArray()));
            // Distance values change every frame: a bounded sample is sufficient.
            DateTime due;if(nextReport.TryGetValue("vehicles:"+key,out due)&&DateTime.UtcNow<due)return;
            nextReport["vehicles:"+key]=DateTime.UtcNow.AddSeconds(5);
            Emit("vehicles:"+key,"safety_vehicles",details,true);
        }catch(Exception e){Error(e);}
    }
    public static void Record(string phase,object point,object unused) {
        try {
            if(phase=="poll") {
                if(DateTime.UtcNow<nextPoll)return;nextPoll=DateTime.UtcNow.AddSeconds(1);
                Type unity=Find("UnityEngine.Object"), type=Find("SpawnPointObject");
                if(unity==null||type==null)return;
                MethodInfo find=unity.GetMethod("FindObjectsOfType",new Type[]{typeof(Type)});
                if(find==null)throw new InvalidOperationException("FindObjectsOfType(Type) unavailable");
                foreach(object source in (IEnumerable)find.Invoke(null,new object[]{type}))Snapshot(source,"snapshot",false);
                return;
            }
            if(phase=="Setting_enter") {
                // A pooled source can reuse both native ID and Unity instance.
                // Diagnostic counters/verdicts belong to the new configuration.
                string key=Identity(point);
                attempts.Remove(key);checks.Remove(key);safety.Remove(key);lastStage.Remove(key);
                nextReport.Remove("poll:"+key);
            }
            if(phase=="attempt_enter") {
                string key=Identity(point);int n;attempts.TryGetValue(key,out n);attempts[key]=n+1;
                lastStage[key]="entered";return;
            }
            if(phase=="attempt_exit") { lastStage[Identity(point)]="returned";return; }
            Snapshot(point,phase,true);
        } catch(Exception e) { Error(e); }
    }
    static void Error(Exception e) {
        try {Emit("error","trace_error",e.GetType().Name+":"+e.Message,false);}catch{}
    }
}
