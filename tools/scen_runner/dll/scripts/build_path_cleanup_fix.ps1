param([Parameter(Mandatory=$true)][string]$ManagedPath)

$ScenDllRoot=Split-Path $PSScriptRoot -Parent
$ScenSourceRoot=Join-Path $ScenDllRoot 'src'
$ScenBuildRoot=Join-Path $ScenDllRoot 'builds'
$ScenScriptsRoot=Join-Path $ScenDllRoot 'scripts'
$ErrorActionPreference='Stop'
function Hash([string]$Path) { if(Test-Path $Path){return (Get-FileHash $Path -Algorithm SHA256).Hash};return $null }
$installed=Join-Path $ManagedPath 'Assembly-CSharp.dll'
$installedHash=Hash $installed
$source=$installed
$activationBackup="$installed.scen-before-activation-trace"
if(Test-Path "$activationBackup.json") {
    $state=Get-Content "$activationBackup.json" -Raw | ConvertFrom-Json
    if($installedHash-eq $state.output_sha256) {
        if((Hash $activationBackup)-ne $state.source_sha256){throw 'Activation base backup hash mismatch.'}
        if((Hash (Join-Path $ManagedPath 'ScenRunner.ActivationTrace.dll'))-ne $state.helper_sha256){throw 'Activation helper hash mismatch.'}
        $source=$activationBackup
    }
}
$sourceHash=Hash $source
if(!$sourceHash){throw 'Simulator assembly not found.'}
$out=Join-Path $ScenBuildRoot 'out_path_cleanup_fix'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$output=Join-Path $out 'Assembly-CSharp.dll'
$cecil=Join-Path $ManagedPath 'Unity.Burst.Cecil.dll'
[Reflection.Assembly]::LoadFrom($cecil) | Out-Null
Add-Type -Path (Join-Path $ScenSourceRoot 'PathCleanupPatch.cs') -ReferencedAssemblies $cecil
$result=[ScenPathCleanupBuilder]::Build($source,$output)
$verified=[ScenPathCleanupBuilder]::Verify($source,$output)
if((Hash $installed)-ne $installedHash-or (Hash $source)-ne $sourceHash){throw 'Simulator assembly changed during build.'}
# Preserve every existing helper dependency, not just the currently known ones.
$helpers=@{}
Get-ChildItem $ManagedPath -Filter 'ScenRunner.*.dll' | Where-Object {$_.Name-ne 'ScenRunner.ActivationTrace.dll'} | ForEach-Object {$helpers[$_.Name]=Hash $_.FullName}
@{schema_version=1;built_utc=[DateTime]::UtcNow.ToString('o');source_sha256=$sourceHash;
  installed_at_build_sha256=$installedHash;output_sha256=(Hash $output);existing_helpers=$helpers;
  behavior='Return only on null/empty path name; valid path deletion and prior fixes unchanged';verification=$verified
} | ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 (Join-Path $out 'manifest.json')
Write-Output $result
Write-Output $verified
Write-Output "Installed assembly unchanged. Path cleanup fix copy: $out"
