
$ScenDllRoot=Split-Path $PSScriptRoot -Parent
$ScenSourceRoot=Join-Path $ScenDllRoot 'src'
$ScenBuildRoot=Join-Path $ScenDllRoot 'builds'
$ScenScriptsRoot=Join-Path $ScenDllRoot 'scripts'
# These installation tests modify only temporary text fixtures.
$ErrorActionPreference='Stop'
$temp=Join-Path ([IO.Path]::GetTempPath()) ('scen_path_install_'+[Guid]::NewGuid().ToString('N'))
function Get-Process {param($Name,$ErrorAction) return $null}
function Hash([string]$Path){return (Get-FileHash $Path -Algorithm SHA256).Hash}
function Assert([bool]$Ok,[string]$Message){if(!$Ok){throw $Message}}
try {
    $managed=Join-Path $temp 'managed';$fixtureDll=Join-Path $temp 'dll';$scripts=Join-Path $fixtureDll 'scripts';$fixtureBuilds=Join-Path $fixtureDll 'builds';$out=Join-Path $fixtureBuilds 'out_path_cleanup_fix'
    New-Item -ItemType Directory -Path $managed,$scripts,$out -Force | Out-Null
    foreach($name in @('install_path_cleanup_fix.ps1','install_trace.ps1','restore_activation_trace.ps1')){Copy-Item (Join-Path $ScenScriptsRoot $name) $scripts}
    $target=Join-Path $managed 'Assembly-CSharp.dll';$runtime=Join-Path $managed 'ScenRunner.RuntimeTrace.dll'
    Set-Content $target 'existing fixes';Set-Content $runtime 'existing helper'
    $base=Hash $target;$runtimeHash=Hash $runtime
    $assembly=Join-Path $out 'Assembly-CSharp.dll';Set-Content $assembly 'existing fixes plus empty path guard'
    $manifest=@{source_sha256=$base;output_sha256=(Hash $assembly);existing_helpers=@{'ScenRunner.RuntimeTrace.dll'=$runtimeHash}}
    $manifest | ConvertTo-Json -Depth 5 | Set-Content (Join-Path $out 'manifest.json')
    $install=Join-Path $scripts 'install_path_cleanup_fix.ps1'
    & $install -ManagedPath $managed
    Assert ((Hash $target)-eq $manifest.output_sha256) 'Fix not installed'
    Assert ((Hash "$target.scen-before-path-cleanup-fix")-eq $base) 'Prior fixes not backed up exactly'
    & $install -ManagedPath $managed
    Assert ((Hash "$target.scen-before-path-cleanup-fix")-eq $base) 'Repeat install overwrote backup'
    Assert ((Hash $runtime)-eq $runtimeHash) 'Existing helper changed'
    & $install -ManagedPath $managed -Restore
    Assert ((Hash $target)-eq $base) 'Fix-only restore lost prior fixes'
    # No-copy rejection when the process check reports a running simulator.
    function Get-Process {param($Name,$ErrorAction) return [pscustomobject]@{ProcessName='Simulator'}}
    $rejected=$false;try{& $install -ManagedPath $managed}catch{$rejected=$true}
    Assert ($rejected-and (Hash $target)-eq $base) 'Running simulator was overwritten'
    function Get-Process {param($Name,$ErrorAction) return $null}
    Set-Content $target 'unknown update';$unknown=Hash $target
    foreach($restore in @($false,$true)) {
        $rejected=$false;try{& $install -ManagedPath $managed -Restore:$restore}catch{$rejected=$true}
        Assert ($rejected-and (Hash $target)-eq $unknown) 'Unknown assembly was overwritten'
    }
    Copy-Item "$target.scen-before-path-cleanup-fix" $target -Force
    Set-Content $runtime 'changed helper';$rejected=$false
    try{& $install -ManagedPath $managed}catch{$rejected=$true}
    Assert ($rejected-and (Hash $target)-eq $base) 'Changed helper was accepted'
    Set-Content $runtime 'existing helper'
    Set-Content $assembly 'tampered output';$rejected=$false
    try{& $install -ManagedPath $managed}catch{$rejected=$true}
    Assert ($rejected-and (Hash $target)-eq $base) 'Tampered build output was accepted'
    Set-Content $assembly 'existing fixes plus empty path guard'
    # Backup tampering must be rejected before either install or restore writes.
    Set-Content "$target.scen-before-path-cleanup-fix" 'tampered backup'
    $rejected=$false;try{& $install -ManagedPath $managed -Restore}catch{$rejected=$true}
    Assert ($rejected-and (Hash $target)-eq $base) 'Tampered backup was restored'
    Copy-Item $target "$target.scen-before-path-cleanup-fix" -Force
    # Full restore unwinds this addition and restores the organizer original.
    $known=Join-Path $fixtureBuilds 'out_static_contact_fix';New-Item -ItemType Directory -Path $known -Force | Out-Null
    Copy-Item $target (Join-Path $known 'Assembly-CSharp.dll')
    $original="$target.scen-trace-original";Set-Content $original 'organizer original'
    (Hash $original) | Set-Content "$original.sha256"
    (Hash $original) | Set-Content (Join-Path $known 'original.sha256')
    & $install -ManagedPath $managed
    # A newer activation diagnostic can be layered on the path fix. Full restore
    # must remove that diagnostic, this fix, then all earlier fixes in that order.
    $activationOut=Join-Path $fixtureBuilds 'out_activation_trace';New-Item -ItemType Directory -Path $activationOut -Force | Out-Null
    $diagnostic=Join-Path $activationOut 'Assembly-CSharp.dll';$activationHelper=Join-Path $activationOut 'ScenRunner.ActivationTrace.dll'
    Set-Content $diagnostic 'path fix plus activation diagnostic';Set-Content $activationHelper 'activation helper'
    @{source_sha256=(Hash $target);output_sha256=(Hash $diagnostic);helper_sha256=(Hash $activationHelper);existing_runtime_helper_sha256=$runtimeHash} |
        ConvertTo-Json | Set-Content (Join-Path $activationOut 'manifest.json')
    Copy-Item $target "$target.scen-before-activation-trace"
    @{source_sha256=(Hash $target);output_sha256=(Hash $diagnostic);helper_sha256=(Hash $activationHelper);
      existing_runtime_helper_sha256=$runtimeHash} | ConvertTo-Json |
        Set-Content "$target.scen-before-activation-trace.json"
    Copy-Item $diagnostic $target -Force
    Copy-Item $activationHelper (Join-Path $managed 'ScenRunner.ActivationTrace.dll') -Force
    & (Join-Path $scripts 'install_trace.ps1') -ManagedPath $managed -Restore
    Assert ((Hash $target)-eq (Hash $original)) 'Full restore did not recover original'
    Assert (!(Test-Path $runtime)) 'Full restore retained diagnostic helper'
    Assert (!(Test-Path (Join-Path $managed 'ScenRunner.ActivationTrace.dll'))) 'Full restore retained activation helper'
    Write-Output 'PASS: temp-only install/repeat/exact rollback/stacked full original restore; process/unknown assembly/changed helper/tampered output and backup refusal without writes.'
} finally {if(Test-Path $temp){Remove-Item $temp -Recurse -Force}}
