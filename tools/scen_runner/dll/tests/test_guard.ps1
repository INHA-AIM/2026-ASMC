param([switch]$ContactFix=$true, [switch]$LifecycleTrace=$true, [switch]$StaticContacts=$true)

$ScenDllRoot=Split-Path $PSScriptRoot -Parent
$ScenSourceRoot=Join-Path $ScenDllRoot 'src'
$ScenBuildRoot=Join-Path $ScenDllRoot 'builds'
$ScenScriptsRoot=Join-Path $ScenDllRoot 'scripts'
$ErrorActionPreference = 'Stop'
$folder = if ($StaticContacts) { 'out_static_contact_fix' } elseif ($LifecycleTrace -and $ContactFix) { 'out_collision_fix' } elseif ($LifecycleTrace) { 'out_collision_trace' } elseif ($ContactFix) { 'out_contact_fix' } else { 'out_fix' }
[Reflection.Assembly]::Load([IO.File]::ReadAllBytes((Join-Path $ScenBuildRoot "$folder/ScenRunner.RuntimeTrace.dll"))) | Out-Null
Add-Type -TypeDefinition @'
using System;
public class MoraiObjectBase {
    public int id; public int instance; public bool fail; public string name="Man6"; public int pedestrianBehavior=7;
    public int get_GetUniqueID() { return id; }
    public int GetInstanceID() { if(fail) throw new Exception("unreadable"); return instance==0?id:instance; }
}
public class MGVehicle : MoraiObjectBase { }
public class PedestrianModel : MoraiObjectBase { }
public class ObstacleObject : MoraiObjectBase { }
public class ScenFakeCollider {
    public object owner;
    public object gameObject { get { return this; } }
    public string name { get { return "man6_renewal"; } }
    public string tag { get { return "Pedestrian"; } }
    public int GetInstanceID() { return 100; }
    public object GetComponentInParent(Type type,bool inactive) { return owner; }
}
public class ObstacleBoxColliderTrigger {
    public MoraiObjectBase cachedOwner;
    public MoraiObjectBase hierarchyOwner;
    public System.Collections.IDictionary m_dicCollObject = new System.Collections.Hashtable();
    public System.Collections.Generic.Dictionary<int,MoraiObjectBase> actorMap = new System.Collections.Generic.Dictionary<int,MoraiObjectBase>();
    public System.Collections.Generic.Dictionary<int,ValueTuple<int,int>> counters = new System.Collections.Generic.Dictionary<int,ValueTuple<int,int>>();
    public object gameObject { get { return this; } }
    public int GetInstanceID() { return 200; }
    public object GetComponentInParent(Type type,bool inactive) { return hierarchyOwner; }
}
public class ScenCollisionData {
    public ObstacleBoxColliderTrigger source;
}
public class ScenStateInfo { public ScenStateId Id=new ScenStateId(); public int ObjectType=1; }
public class ScenStateId { public string Value="Ego"; }
public class ScenVehicleState { public System.Collections.Generic.List<string> CollisionObjects=new System.Collections.Generic.List<string>(); }
public class ScenActorState { public ScenStateInfo ActorInfo=new ScenStateInfo(); public ScenVehicleState VehicleState=new ScenVehicleState(); }
public class ProjectManager {
    private static ProjectManager singleton=new ProjectManager();
    public static ProjectManager instance { get { return singleton; } }
    public System.Collections.Generic.Dictionary<UnityEngine.GameObject,MoraiObjectBase> objects=new System.Collections.Generic.Dictionary<UnityEngine.GameObject,MoraiObjectBase>();
    public MoraiObjectBase GetObjectBase(UnityEngine.GameObject key) {
        MoraiObjectBase result; return objects.TryGetValue(key,out result)?result:null;
    }
}
namespace UnityEngine {
    public class GameObject {
        public object GetComponentInParent(Type type,bool inactive) { return null; }
        public int GetInstanceID() { return 999; }
    }
    public class Object {
        public static object[] triggers = new object[0];
        public static void SetTrigger(ObstacleBoxColliderTrigger trigger) { triggers=new object[]{trigger}; }
        public static object[] FindObjectsOfType(Type type,bool inactive) { return triggers; }
    }
    public class BoxCollider { public string bounds="test"; }
    public static class Debug {
        public static System.Collections.Generic.List<string> records = new System.Collections.Generic.List<string>();
        public static void Log(object message) { records.Add(message.ToString()); }
    }
}
'@
$a = New-Object MoraiObjectBase; $a.id=1
$b = New-Object MoraiObjectBase; $b.id=2
$bad = New-Object MoraiObjectBase; $bad.id=1; $bad.fail=$true
$one = New-Object ScenFakeCollider; $one.owner=$a
$same = New-Object ScenFakeCollider; $same.owner=$a
$different = New-Object ScenFakeCollider; $different.owner=$b
$unknown = New-Object ScenFakeCollider
$unreadable = New-Object ScenFakeCollider; $unreadable.owner=$bad
if (![ScenRunnerRuntimeTrace]::SkipSelfCleanup($one,$same)) { throw 'Same owner was not excluded.' }
if ([ScenRunnerRuntimeTrace]::SkipSelfCleanup($one,$different)) { throw 'Different owner was incorrectly excluded.' }
if ([ScenRunnerRuntimeTrace]::SkipSelfCleanup($one,$unknown)) { throw 'Unknown owner was incorrectly excluded.' }
if ([ScenRunnerRuntimeTrace]::SkipSelfCleanup($one,$unreadable)) { throw 'Unreadable owner was incorrectly excluded.' }
if ([ScenRunnerRuntimeTrace]::SkipSelfCleanup($null,$same)) { throw 'Null subject was incorrectly excluded.' }
Write-Output 'PASS: same owner excluded; different, unknown, unreadable and null owners preserve original cleanup.'
$pet=New-Object MoraiObjectBase; $pet.id=3; $pet.name='Ped_Pet1(Clone)'
if ([ScenRunnerRuntimeTrace]::SpawnBehavior($pet) -ne 7) { throw 'PET behavior was not preserved.' }
if ([ScenRunnerRuntimeTrace]::SpawnBehavior($a) -ne 0) { throw 'Non-Pet1 model behavior changed.' }
$pet.pedestrianBehavior=1
if ([ScenRunnerRuntimeTrace]::SpawnBehavior($pet) -ne 0) { throw 'Non-PET preset behavior changed.' }
if ([ScenRunnerRuntimeTrace]::SpawnBehavior($null) -ne 0) { throw 'Null model behavior changed.' }
Write-Output 'PASS: Ped_Pet1 PET=7 preserved; other models and behavior presets unchanged.'
[UnityEngine.Debug]::records.Clear()
$trigger = New-Object ObstacleBoxColliderTrigger
$trigger.hierarchyOwner=$a; $trigger.cachedOwner=$b
$source = New-Object ScenCollisionData; $source.source=$trigger
[ScenRunnerRuntimeTrace]::Record('collision_source',$source,$null)
if ([UnityEngine.Debug]::records.Count -ne 1) { throw 'Collision source log missing.' }
if (![UnityEngine.Debug]::records[0].Contains('hierarchyOwner=1;cachedOwner=2;')) { throw 'Owner identity mismatch was not exposed.' }
[ScenRunnerRuntimeTrace]::Record('collision_source',$source,$null)
if ([UnityEngine.Debug]::records.Count -ne 1) { throw 'Unchanged source log was repeated.' }
$trigger.m_dicCollObject.Add($different,$null)
[ScenRunnerRuntimeTrace]::Record('collision_source',$source,$null)
if ([UnityEngine.Debug]::records.Count -ne 2) { throw 'Changed contact membership was not logged.' }
Write-Output 'PASS: collision source exposes hierarchy/cached owners; repeats suppressed; changed contacts logged.'
if ($LifecycleTrace) {
    [UnityEngine.Debug]::records.Clear()
    $originalCount=$trigger.m_dicCollObject.Count
    [ScenRunnerRuntimeTrace]::Record('contact_EnableAccurateCollisionTrigger_enter',$trigger,$true)
    if ([UnityEngine.Debug]::records.Count -ne 1) { throw 'Lifecycle snapshot missing.' }
    if ($trigger.m_dicCollObject.Count -ne $originalCount) { throw 'Diagnostic modified contact dictionary.' }
    if (![UnityEngine.Debug]::records[0].Contains('m_dicCollObject=')) { throw 'Dictionary snapshot missing.' }
    $state=New-Object ScenActorState
    $state.VehicleState.CollisionObjects.Add('Ego')
    [ScenRunnerRuntimeTrace]::RecordActorStateReturn($state)
    $beforeCount=[UnityEngine.Debug]::records.Count
    [ScenRunnerRuntimeTrace]::RecordActorStateReturn($state)
    if ([UnityEngine.Debug]::records.Count -ne $beforeCount) { throw 'Unchanged actor-state return repeated.' }
    $state.VehicleState.CollisionObjects.Add('1')
    [ScenRunnerRuntimeTrace]::RecordActorStateReturn($state)
    if ([UnityEngine.Debug]::records.Count -ne ($beforeCount+1)) { throw 'Contact change was not captured.' }
    if (![UnityEngine.Debug]::records[-1].Contains('contacts=1,Ego')) { throw 'Original contact IDs were not preserved.' }
    if ($state.VehicleState.CollisionObjects.Count -ne 2) { throw 'Return trace altered original state.' }
    $state.VehicleState.CollisionObjects.Remove('1') | Out-Null
    $wrapped=[Tuple]::Create($true,$state)
    [ScenRunnerRuntimeTrace]::RecordActorStateReturn($wrapped)
    if (![UnityEngine.Debug]::records[-1].Contains('ok=True;id=Ego;')) { throw 'Wrapped actor-state return was not decoded.' }
    [ScenRunnerRuntimeTrace]::RecordActorStateReturn($null)
    Write-Output 'PASS: lifecycle and state-return diagnostics read only; changed contacts logged; repeats suppressed; null safe.'
}
if ($ContactFix) {
    $victim=New-Object MGVehicle; $victim.id=7; $victim.instance=700
    $otherVehicle=New-Object MGVehicle; $otherVehicle.id=7; $otherVehicle.instance=701
    $ego=New-Object MGVehicle; $ego.id=0; $ego.instance=900
    $victimKey=New-Object ScenFakeCollider; $victimKey.owner=$victim
    $otherKey=New-Object ScenFakeCollider; $otherKey.owner=$otherVehicle
    $egoKey=New-Object ScenFakeCollider; $egoKey.owner=$ego
    $peer=New-Object ObstacleBoxColliderTrigger; $peer.hierarchyOwner=$ego
    $peer.m_dicCollObject.Add($victimKey,$null)
    $peer.m_dicCollObject.Add($otherKey,$null)
    $peer.m_dicCollObject.Add($egoKey,$null)
    $registryKey=New-Object UnityEngine.GameObject
    [ProjectManager]::instance.objects.Add($registryKey,$victim)
    $peer.m_dicCollObject.Add($registryKey,$null)
    [UnityEngine.Object]::SetTrigger($peer)
    [ScenRunnerRuntimeTrace]::ClearRetiringVehicleContacts($victim)
    if ($peer.m_dicCollObject.Contains($victimKey)) { throw ('Retired vehicle reference remains. ' + ([UnityEngine.Debug]::records -join "`n")) }
    if (!$peer.m_dicCollObject.Contains($otherKey)) { throw 'Other vehicle with same numeric ID was removed.' }
    if (!$peer.m_dicCollObject.Contains($egoKey)) { throw 'Ego reference was removed.' }
    if ($peer.m_dicCollObject.Contains($registryKey)) { throw 'Registry-resolved retired reference remains.' }
    [ScenRunnerRuntimeTrace]::ClearRetiringVehicleContacts($ego)
    if (!$peer.m_dicCollObject.Contains($egoKey)) { throw 'Ego retirement was not excluded.' }
    [ScenRunnerRuntimeTrace]::ClearRetiringVehicleContacts($victim)
    if ($peer.m_dicCollObject.Count -ne 2) { throw 'Repeated cleanup changed remaining entries.' }
    Write-Output 'PASS: retired vehicle references removed; ego and other instances preserved; repeated cleanup safe.'
    $deer=New-Object PedestrianModel; $deer.id=1; $deer.instance=1001
    $otherDeer=New-Object PedestrianModel; $otherDeer.id=1; $otherDeer.instance=1002
    $deerKey=New-Object ScenFakeCollider; $deerKey.owner=$deer
    $peer.actorMap.Add(1,$deer)
    $peer.actorMap.Add(2,$otherDeer)
    $peer.actorMap.Add(0,$ego)
    $pair=New-Object 'System.ValueTuple[int,int]' -ArgumentList 10,10
    $peer.counters.Add(1,$pair); $peer.counters.Add(2,$pair); $peer.counters.Add(0,$pair)
    $peer.m_dicCollObject.Add($deerKey,$null)
    [ScenRunnerRuntimeTrace]::ClearRetiringActorContacts($deer)
    if ($peer.m_dicCollObject.Contains($deerKey) -or $peer.actorMap.ContainsKey(1) -or $peer.counters.ContainsKey(1)) {
        throw 'Pedestrian references or mesh counter remain.'
    }
    if (!$peer.actorMap.ContainsKey(2) -or !$peer.counters.ContainsKey(2) -or !$peer.counters.ContainsKey(0)) {
        throw 'Other instance or ego counter removed.'
    }
    $peer.actorMap.Add(1,$otherDeer); $peer.counters.Add(1,$pair)
    [ScenRunnerRuntimeTrace]::ClearRetiringActorContacts($deer)
    if (!$peer.actorMap.ContainsKey(1) -or !$peer.counters.ContainsKey(1)) { throw 'Same ID of another actor was purged.' }
    [ScenRunnerRuntimeTrace]::ClearRetiringActorContacts($ego)
    if (!$peer.actorMap.ContainsKey(0) -or !$peer.counters.ContainsKey(0)) { throw 'Ego was purged.' }
    Write-Output 'PASS: pedestrian reference/mapping/counter removed together; same-ID other instance and ego preserved.'
}
if ($StaticContacts) {
    $bollard=New-Object ObstacleObject; $bollard.id=1; $bollard.instance=113558
    $otherBollard=New-Object ObstacleObject; $otherBollard.id=1; $otherBollard.instance=113559
    $ego=New-Object MGVehicle; $ego.id=0; $ego.instance=900
    $key=New-Object ScenFakeCollider; $key.owner=$bollard
    $otherKey=New-Object ScenFakeCollider; $otherKey.owner=$otherBollard
    $egoKey=New-Object ScenFakeCollider; $egoKey.owner=$ego
    $peer=New-Object ObstacleBoxColliderTrigger; $peer.hierarchyOwner=$ego
    $peer.m_dicCollObject.Add($key,$null); $peer.m_dicCollObject.Add($otherKey,$null); $peer.m_dicCollObject.Add($egoKey,$null)
    $peer.actorMap.Add(1,$bollard); $peer.actorMap.Add(2,$otherBollard); $peer.actorMap.Add(0,$ego)
    $six=New-Object 'System.ValueTuple[int,int]' -ArgumentList 6,6
    $peer.counters.Add(1,$six); $peer.counters.Add(2,$six); $peer.counters.Add(0,$six)
    $registryKey=New-Object UnityEngine.GameObject
    [ProjectManager]::instance.objects.Add($registryKey,$bollard)
    $peer.m_dicCollObject.Add($registryKey,$null)
    [UnityEngine.Object]::SetTrigger($peer)
    [ScenRunnerRuntimeTrace]::ClearRetiringActorContacts($bollard)
    if (!$peer.counters.ContainsKey(1)) { throw 'Legacy vehicle/pedestrian admission changed.' }
    # This models the actual GameObject argument of DestroyObstacle -> PushToPool.
    [ScenRunnerRuntimeTrace]::ClearRetiringStaticContacts($registryKey)
    if ($peer.m_dicCollObject.Contains($key) -or $peer.m_dicCollObject.Contains($registryKey) -or
        $peer.actorMap.ContainsKey(1) -or $peer.counters.ContainsKey(1)) { throw 'Static stale reference or counter 6 remains.' }
    if (!$peer.m_dicCollObject.Contains($otherKey) -or !$peer.m_dicCollObject.Contains($egoKey) -or
        !$peer.counters.ContainsKey(2) -or !$peer.counters.ContainsKey(0)) { throw 'Other instance or ego was removed.' }
    [ScenRunnerRuntimeTrace]::ClearRetiringStaticContacts($bollard)
    if ($peer.m_dicCollObject.Count -ne 2 -or $peer.counters.Count -ne 2) { throw 'Repeated static cleanup changed unrelated contacts.' }
    # The same pooled object returns as ID 1 with no old entry. A subsequent
    # new physical contact can add a fresh record instead of inheriting 6.
    if ($peer.actorMap.ContainsKey(1)) { throw 'Same-instance reuse retained old mapping.' }
    $peer.actorMap.Add(1,$bollard)
    $one=New-Object 'System.ValueTuple[int,int]' -ArgumentList 1,1
    $peer.counters.Add(1,$one); $peer.m_dicCollObject.Add($key,$null)
    if ($peer.counters[1].Item1 -ne 1) { throw 'Fresh contact inherited the old counter.' }
    [ScenRunnerRuntimeTrace]::ClearRetiringStaticContacts($bollard)
    $peer.actorMap.Add(1,$otherBollard); $peer.counters.Add(1,$one)
    [ScenRunnerRuntimeTrace]::ClearRetiringStaticContacts($bollard)
    if (!$peer.actorMap.ContainsKey(1) -or !$peer.counters.ContainsKey(1)) { throw 'Reused ID belonging to another instance was removed.' }
    [ScenRunnerRuntimeTrace]::ClearRetiringStaticContacts($ego)
    $invalid=New-Object ObstacleObject; $invalid.id=0; $invalid.instance=900
    [ScenRunnerRuntimeTrace]::ClearRetiringStaticContacts($invalid)
    [ScenRunnerRuntimeTrace]::ClearRetiringStaticContacts($null)
    if (!$peer.counters.ContainsKey(0) -or !$peer.counters.ContainsKey(1)) { throw 'Null/ego/invalid ID cleanup changed records.' }
    if (!([UnityEngine.Debug]::records -join "`n").Contains('phase=retired_static_contacts_removed actor=1')) {
        throw 'Static cleanup verification log missing.'
    }
    Write-Output 'PASS: static stale counter 6 removed before reuse; fresh contact starts at 1; registry-only pool arguments supported; other instances and ego preserved.'
}
