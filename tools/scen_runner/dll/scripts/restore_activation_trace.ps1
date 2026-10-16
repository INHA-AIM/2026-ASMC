param([Parameter(Mandatory=$true)][string]$ManagedPath)
$ErrorActionPreference='Stop'
if(Get-Process -Name Simulator -ErrorAction SilentlyContinue){throw 'Close MORAI Simulator first. No files were changed.'}
$target=Join-Path $ManagedPath 'Assembly-CSharp.dll'
$helper=Join-Path $ManagedPath 'ScenRunner.ActivationTrace.dll'
$backup="$target.scen-before-activation-trace"
$helperBackup="$helper.scen-before-activation-trace"
$stateFile="$backup.json"
function Hash([string]$Path){if(Test-Path $Path){return (Get-FileHash $Path -Algorithm SHA256).Hash};return $null}
    if(!(Test-Path $stateFile)-or !(Test-Path $backup)){throw 'Activation pre-install backup/metadata not found. No files changed.'}
    $state=Get-Content $stateFile -Raw | ConvertFrom-Json
    if((Hash $backup)-ne $state.source_sha256){throw 'Activation backup hash mismatch. No files changed.'}
    $current=Hash $target
    if($current-ne $state.output_sha256-and $current-ne $state.source_sha256){throw 'Installed assembly changed after activation trace. Refusing to overwrite it.'}
    if($state.previous_helper_sha256-and (Hash $helperBackup)-ne $state.previous_helper_sha256){throw 'Previous activation helper backup mismatch.'}
    if($current-eq $state.output_sha256-and (Hash $helper)-ne $state.helper_sha256){throw 'Installed activation helper hash mismatch.'}
    if((Hash (Join-Path $ManagedPath 'ScenRunner.RuntimeTrace.dll'))-ne $state.existing_runtime_helper_sha256){throw 'Prior runtime helper changed after activation installation. No files changed.'}
    Copy-Item $backup $target -Force
    if($state.previous_helper_sha256){Copy-Item $helperBackup $helper -Force}
    elseif(Test-Path $helper){Remove-Item $helper}
    Write-Output 'Activation diagnosis removed. Exact pre-install assembly restored; prior fixes retained. Restart MORAI.'
    exit
