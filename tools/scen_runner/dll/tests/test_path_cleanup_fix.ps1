param([Parameter(Mandatory=$true)][string]$ManagedPath)

$ScenDllRoot=Split-Path $PSScriptRoot -Parent
$ScenSourceRoot=Join-Path $ScenDllRoot 'src'
$ScenBuildRoot=Join-Path $ScenDllRoot 'builds'
$ScenScriptsRoot=Join-Path $ScenDllRoot 'scripts'
$ErrorActionPreference='Stop'
$temp=Join-Path ([IO.Path]::GetTempPath()) ('scen_path_guard_'+[Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $temp | Out-Null
try {
    $cecil=Join-Path $ManagedPath 'Unity.Burst.Cecil.dll'
    [Reflection.Assembly]::LoadFrom($cecil) | Out-Null
    Add-Type -Path (Join-Path $ScenSourceRoot 'PathCleanupPatch.cs') -ReferencedAssemblies $cecil
    $source=Join-Path $temp 'fixture.dll';$output=Join-Path $temp 'patched.dll'
    $provider=New-Object Microsoft.CSharp.CSharpCodeProvider
    $options=New-Object System.CodeDom.Compiler.CompilerParameters
    $options.GenerateExecutable=$false;$options.OutputAssembly=$source
    $fixture=@'
using System;
using System.Collections.Generic;
namespace UnityEngine {
 public class GameObject { public string name; public bool deleted; public GameObject(string value){name=value;} }
 public class Object { public static int deletions; public static void DestroyImmediate(object target){((GameObject)target).deleted=true;deletions++;} }
 public class Transform {
  public GameObject gameObject; public Dictionary<string,Transform> children=new Dictionary<string,Transform>();
  public Transform(string name){gameObject=new GameObject(name);}
  public Transform Find(string name){if(name==null||name=="")return this;Transform found;return children.TryGetValue(name,out found)?found:null;}
 }
}
namespace SWS {
 public class WaypointManager {
  public static WaypointManager instance=new WaypointManager();
  public UnityEngine.Transform transform=new UnityEngine.Transform("WaypointManager");
  public static int nativeCalls;
  public static void DeletePathManager(string name){
   nativeCalls++;
   UnityEngine.Transform target=instance.transform.Find(name);
   if(target==null)return;
   UnityEngine.Object.DestroyImmediate(target.gameObject);
  }
  public static bool RootAlive(){return !instance.transform.gameObject.deleted;}
  public static void Add(string name){instance.transform.children[name]=new UnityEngine.Transform(name);}
  public static bool Alive(string name){return !instance.transform.children[name].gameObject.deleted;}
 }
}
public class PriorFix {
 public static int Value(){return 72;}
 public static int Branches(int n){try{if(n<0)return -1;if(n==0)return 0;return n+2;}catch{return -3;}}
}
'@
    $compiled=$provider.CompileAssemblyFromSource($options,[string[]]@($fixture))
    if($compiled.Errors.HasErrors){throw ($compiled.Errors | Out-String)}
    [ScenPathCleanupBuilder]::Build($source,$output) | Write-Output
    [ScenPathCleanupBuilder]::Verify($source,$output) | Write-Output
    $assembly=[Reflection.Assembly]::Load([IO.File]::ReadAllBytes($output))
    $manager=$assembly.GetType('SWS.WaypointManager')
    $delete=$manager.GetMethod('DeletePathManager')
    $manager.GetMethod('Add').Invoke($null,[object[]]@('PedPath - 3')) | Out-Null
    $manager.GetMethod('Add').Invoke($null,[object[]]@('PedPath - 4')) | Out-Null
    foreach($name in @($null,'')) {
        $delete.Invoke($null,[object[]]@($name)) | Out-Null
        if(!$manager.GetMethod('RootAlive').Invoke($null,$null)){throw 'Empty name deleted the manager.'}
        if(!$manager.GetMethod('Alive').Invoke($null,[object[]]@('PedPath - 3'))){throw 'Empty name deleted Boy1 path.'}
    }
    if($manager.GetField('nativeCalls').GetValue($null)-ne 0){throw 'Null/empty names entered native cleanup.'}
    $delete.Invoke($null,[object[]]@('PedPath - 3')) | Out-Null
    if($manager.GetMethod('Alive').Invoke($null,[object[]]@('PedPath - 3'))){throw 'Valid path was not deleted.'}
    if(!$manager.GetMethod('Alive').Invoke($null,[object[]]@('PedPath - 4'))){throw 'Other path was deleted.'}
    $delete.Invoke($null,[object[]]@('missing')) | Out-Null
    if(!$manager.GetMethod('RootAlive').Invoke($null,$null)){throw 'Missing nonempty name deleted manager.'}
    if($assembly.GetType('PriorFix').GetMethod('Value').Invoke($null,$null)-ne 72){throw 'Prior fix changed.'}
    $rejected=$false
    try{[ScenPathCleanupBuilder]::Build($output,(Join-Path $temp 'double.dll')) | Out-Null}catch{$rejected=$true}
    if(!$rejected){throw 'Duplicate guard was accepted.'}
    # Verification must reject unrelated changes, including an existing fix body.
    $modified=Join-Path $temp 'tampered.dll'
    $a=[Mono.Cecil.AssemblyDefinition]::ReadAssembly($output)
    try {
        $type=$a.MainModule.Types | Where-Object {$_.Name-eq 'PriorFix'}
        $method=$type.Methods | Where-Object {$_.Name-eq 'Value'}
        $instruction=$method.Body.Instructions | Where-Object {$_.OpCode.Code-eq 'Ldc_I4_S'} | Select-Object -First 1
        $instruction.Operand=[sbyte]71
        $a.Write($modified)
    } finally {$a.Dispose()}
    $rejected=$false
    try{[ScenPathCleanupBuilder]::Verify($source,$modified) | Out-Null}catch{$rejected=$true}
    if(!$rejected){throw 'Unrelated native behavior change passed verification.'}
    Write-Output 'PASS: null/empty names preserve manager and Boy1; valid deletion/missing path/other path unchanged; prior fix retained; duplicate and unrelated mutations rejected.'
} finally {if(Test-Path $temp){Remove-Item $temp -Recurse -Force}}
