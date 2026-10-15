param([Parameter(Mandatory=$true)][string]$ManagedPath, [switch]$FixSelf, [switch]$FixContacts, [switch]$LifecycleTrace, [switch]$SpawnTrace, [switch]$StaticContacts)

$ScenDllRoot=Split-Path $PSScriptRoot -Parent
$ScenSourceRoot=Join-Path $ScenDllRoot 'src'
$ScenBuildRoot=Join-Path $ScenDllRoot 'builds'
$ScenScriptsRoot=Join-Path $ScenDllRoot 'scripts'
$ErrorActionPreference = 'Stop'
$folder = if ($StaticContacts) { 'out_static_contact_fix' } elseif ($SpawnTrace) { 'out_spawn_trace' } elseif ($LifecycleTrace -and $FixContacts) { 'out_collision_fix' } elseif ($LifecycleTrace) { 'out_collision_trace' } elseif ($FixContacts) { 'out_contact_fix' } elseif ($FixSelf) { 'out_fix' } else { 'out' }
$out = Join-Path $ScenBuildRoot $folder
New-Item -ItemType Directory -Force -Path $out | Out-Null
$source = Join-Path $ManagedPath 'Assembly-CSharp.dll'
$backup = Join-Path $ManagedPath 'Assembly-CSharp.dll.scen-trace-original'
if (Test-Path $backup) { $source = $backup }
$before = (Get-FileHash $source -Algorithm SHA256).Hash
$helper = Join-Path $out 'ScenRunner.RuntimeTrace.dll'
$outputAssembly = Join-Path $out 'Assembly-CSharp.dll'
if ($FixSelf -and (Test-Path $outputAssembly)) {
    (Get-FileHash $outputAssembly -Algorithm SHA256).Hash | Add-Content (Join-Path $out 'previous.sha256')
}
if (Test-Path $helper) { Remove-Item $helper }
Add-Type -Path @((Join-Path $ScenSourceRoot 'TraceHooks.cs'),(Join-Path $ScenSourceRoot 'SpawnTrace.cs')) -OutputAssembly $helper
$cecil = Join-Path $ManagedPath 'Unity.Burst.Cecil.dll'
[Reflection.Assembly]::LoadFrom($cecil) | Out-Null
try { Add-Type -Path (Join-Path $ScenSourceRoot 'PatchTrace.cs') -ReferencedAssemblies $cecil }
catch {
    if ($_.Exception.LoaderExceptions) { $_.Exception.LoaderExceptions | ForEach-Object { Write-Output $_.Message } }
    throw
}
$result = [ScenTraceBuilder]::Build($source, $helper, (Join-Path $out 'Assembly-CSharp.dll'), [bool]$FixSelf, [bool]$FixContacts, [bool]$LifecycleTrace, [bool]$SpawnTrace, [bool]$StaticContacts)
$verified = [ScenTraceBuilder]::Verify($source, (Join-Path $out 'Assembly-CSharp.dll'), [bool]$FixSelf, [bool]$FixContacts, [bool]$StaticContacts)
if ((Get-FileHash $source -Algorithm SHA256).Hash -ne $before) { throw 'Original DLL changed unexpectedly.' }
$before | Set-Content (Join-Path $out 'original.sha256')
Write-Output $result
Write-Output $verified
Write-Output "Original unchanged. Diagnostic copies: $out"
