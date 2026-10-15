param([Parameter(Mandatory=$true)][string]$ManagedPath,[switch]$Restore)

$ScenDllRoot=Split-Path $PSScriptRoot -Parent
$ScenSourceRoot=Join-Path $ScenDllRoot 'src'
$ScenBuildRoot=Join-Path $ScenDllRoot 'builds'
$ScenScriptsRoot=Join-Path $ScenDllRoot 'scripts'
$ErrorActionPreference='Stop'
if(Get-Process -Name Simulator -ErrorAction SilentlyContinue){throw 'Close MORAI Simulator first. No files were changed.'}
function Hash([string]$Path) { if(Test-Path $Path){return (Get-FileHash $Path -Algorithm SHA256).Hash};return $null }
function Check-Helpers($Expected) {
    $current=@{}
    Get-ChildItem $ManagedPath -Filter 'ScenRunner.*.dll' | ForEach-Object {$current[$_.Name]=Hash $_.FullName}
    $count=0
    foreach($property in $Expected.PSObject.Properties) {
        $count++
        if($current[$property.Name]-ne $property.Value){throw "Existing helper changed: $($property.Name). No files changed."}
    }
    if($current.Count-ne $count){throw 'Helper dependencies changed. Remove the activation diagnostic or rebuild first. No files changed.'}
}
$target=Join-Path $ManagedPath 'Assembly-CSharp.dll'
$backup="$target.scen-before-path-cleanup-fix"
$stateFile="$backup.json"
if($Restore) {
    if(!(Test-Path $stateFile)-or !(Test-Path $backup)){throw 'Path cleanup pre-install backup/metadata not found.'}
    $state=Get-Content $stateFile -Raw | ConvertFrom-Json
    $current=Hash $target
    if((Hash $backup)-ne $state.source_sha256){throw 'Path cleanup backup hash mismatch. No files changed.'}
    if($current-ne $state.output_sha256-and $current-ne $state.source_sha256){throw 'Installed assembly changed after path cleanup fix. Refusing to overwrite.'}
    Check-Helpers $state.existing_helpers
    Copy-Item $backup $target -Force
    Write-Output 'Path cleanup fix removed. Exact pre-install assembly and existing helpers retained. Restart MORAI.'
    exit
}
$out=Join-Path $ScenBuildRoot 'out_path_cleanup_fix'
$manifestPath=Join-Path $out 'manifest.json'
if(!(Test-Path $manifestPath)){throw 'Run build-path-cleanup-fix first.'}
$manifest=Get-Content $manifestPath -Raw | ConvertFrom-Json
$assembly=Join-Path $out 'Assembly-CSharp.dll'
if((Hash $assembly)-ne $manifest.output_sha256){throw 'Build output hash mismatch. No files changed.'}
$current=Hash $target
if($current-ne $manifest.source_sha256-and $current-ne $manifest.output_sha256){throw 'Installed assembly differs from fix base. Run restore-activation-trace first if its diagnosis is installed; otherwise rebuild. No files changed.'}
Check-Helpers $manifest.existing_helpers
if(Test-Path $stateFile) {
    $state=Get-Content $stateFile -Raw | ConvertFrom-Json
    if((Hash $backup)-ne $manifest.source_sha256-or $state.source_sha256-ne $manifest.source_sha256){throw 'Path cleanup backup belongs to another base assembly. No files changed.'}
} else {
    if($current-ne $manifest.source_sha256-or (Test-Path $backup)){throw 'A verified pre-install backup is required. No files changed.'}
    Copy-Item $target $backup
}
if($current-eq $manifest.output_sha256) {
    if($state.output_sha256-ne $manifest.output_sha256){throw 'Installed fix does not match its saved installation metadata.'}
    Write-Output 'Path cleanup fix already installed. Original pre-install backup retained. Restart MORAI.'
    exit
}
@{source_sha256=$manifest.source_sha256;output_sha256=$manifest.output_sha256;existing_helpers=$manifest.existing_helpers} |
    ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 $stateFile
try { Copy-Item $assembly $target -Force }
catch { Copy-Item $backup $target -Force;throw }
Write-Output 'Empty path-name cleanup guard installed. Existing self-contact/Pet1/contact fixes and helpers retained. Restart MORAI; verify movement with run_validation.sh.'
