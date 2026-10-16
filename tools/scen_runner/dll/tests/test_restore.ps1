# Final contact fix installation/rollback uses only temporary text fixtures.
param([string]$ManagedPath)

$ScenDllRoot=Split-Path $PSScriptRoot -Parent
$ScenSourceRoot=Join-Path $ScenDllRoot 'src'
$ScenBuildRoot=Join-Path $ScenDllRoot 'builds'
$ScenScriptsRoot=Join-Path $ScenDllRoot 'scripts'
$ErrorActionPreference='Stop'
$temp=Join-Path ([IO.Path]::GetTempPath()) ('scen_restore_'+[Guid]::NewGuid().ToString('N'))
function Hash([string]$Path){return (Get-FileHash $Path -Algorithm SHA256).Hash}
function Assert([bool]$Ok,[string]$Message){if(!$Ok){throw $Message}}
function Invoke-Fixture([string]$Switch) {
    $command="function Get-Process {param(`$Name) return @()}; & '"+$installer.Replace("'","''")+
             "' -ManagedPath '"+$managed.Replace("'","''")+"' "+$Switch
    $priorPreference=$ErrorActionPreference;$ErrorActionPreference='Continue'
    $output=& powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass -Command $command 2>&1
    $code=$LASTEXITCODE;$ErrorActionPreference=$priorPreference
    return [pscustomobject]@{code=$code;output=($output | Out-String)}
}
function Success([string]$Switch) {
    $result=Invoke-Fixture $Switch
    if($result.code-ne 0){throw $result.output}
}
try {
    $managed=Join-Path $temp 'managed';$fixtureDll=Join-Path $temp 'dll';$scripts=Join-Path $fixtureDll 'scripts'
    $fixtureBuilds=Join-Path $fixtureDll 'builds';$out=Join-Path $fixtureBuilds 'out_static_contact_fix'
    New-Item -ItemType Directory -Path $managed,$scripts,$out -Force | Out-Null
    foreach($name in @('install_trace.ps1','install_path_cleanup_fix.ps1','restore_activation_trace.ps1')) {
        Copy-Item (Join-Path $ScenScriptsRoot $name) $scripts
    }
    $installer=Join-Path $scripts 'install_trace.ps1';$target=Join-Path $managed 'Assembly-CSharp.dll'
    $helper=Join-Path $managed 'ScenRunner.RuntimeTrace.dll';$backup="$target.scen-trace-original"
    Set-Content $target 'organizer original';$original=Hash $target
    $assembly=Join-Path $out 'Assembly-CSharp.dll';Set-Content $assembly 'final combined contact fix'
    Set-Content (Join-Path $out 'ScenRunner.RuntimeTrace.dll') 'final runtime helper'
    $outputHash=Hash $assembly;$original | Set-Content (Join-Path $out 'original.sha256')
    $install='-FixSelf -FixContacts -LifecycleTrace -SpawnTrace -StaticContacts'
    Success $install
    Assert ((Hash $target)-eq $outputHash-and (Hash $backup)-eq $original) 'Final fix/original backup mismatch'
    $prior="$target.scen-before-static-contact-fix";$priorHash=Hash $prior
    Success $install
    Assert ((Hash $prior)-eq $priorHash) 'Repeated install overwrote pre-install backup'
    Success '-RestoreStaticContacts'
    Assert ((Hash $target)-eq $original-and !(Test-Path $helper)) 'Fix-only rollback failed'
    Success $install;Success '-Restore'
    Assert ((Hash $target)-eq $original-and !(Test-Path $helper)) 'Full original restore failed'
    Write-Output 'PASS: final contact fix install/repeat/exact prior rollback/full original restore.'
    Set-Content $target 'unknown simulator update';$unknown=Hash $target
    foreach($switch in @($install,'-Restore')) {
        $result=Invoke-Fixture $switch
        Assert ($result.code-ne 0-and (Hash $target)-eq $unknown) 'Unknown simulator binary was overwritten'
    }
    Copy-Item $assembly $target -Force
    Set-Content $backup 'tampered original backup';$before=Hash $target
    $result=Invoke-Fixture '-Restore'
    Assert ($result.code-ne 0-and (Hash $target)-eq $before) 'Tampered original backup was restored'
    Write-Output 'PASS: unknown versions and tampered backups rejected without writes.'
} finally {if(Test-Path $temp){Remove-Item $temp -Recurse -Force}}
