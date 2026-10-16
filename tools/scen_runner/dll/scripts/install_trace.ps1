param([Parameter(Mandatory=$true)][string]$ManagedPath, [switch]$Restore, [switch]$FixSelf, [switch]$FixContacts, [switch]$LifecycleTrace, [switch]$RestoreLifecycle, [switch]$RestoreContactFix, [switch]$SpawnTrace, [switch]$RestoreSpawnTrace, [switch]$StaticContacts, [switch]$RestoreStaticContacts)

$ScenDllRoot=Split-Path $PSScriptRoot -Parent
$ScenSourceRoot=Join-Path $ScenDllRoot 'src'
$ScenBuildRoot=Join-Path $ScenDllRoot 'builds'
$ScenScriptsRoot=Join-Path $ScenDllRoot 'scripts'
$ErrorActionPreference = 'Stop'
if (Get-Process -Name Simulator -ErrorAction SilentlyContinue) { throw 'Close MORAI Simulator first. No files were changed.' }
$target = Join-Path $ManagedPath 'Assembly-CSharp.dll'
# A diagnostic overlay is built from the installed fix assembly. Unwind it first
# when the user explicitly asks for a full original restore.
$activationState = "$target.scen-before-activation-trace.json"
if ($Restore -and (Test-Path $activationState)) {
    $state = Get-Content $activationState -Raw | ConvertFrom-Json
    if ((Get-FileHash $target -Algorithm SHA256).Hash -eq $state.output_sha256) {
        & (Join-Path $ScenScriptsRoot 'restore_activation_trace.ps1') -ManagedPath $ManagedPath
    }
}
$pathCleanupState = "$target.scen-before-path-cleanup-fix.json"
if ($Restore -and (Test-Path $pathCleanupState)) {
    $state = Get-Content $pathCleanupState -Raw | ConvertFrom-Json
    if ((Get-FileHash $target -Algorithm SHA256).Hash -eq $state.output_sha256) {
        & (Join-Path $ScenScriptsRoot 'install_path_cleanup_fix.ps1') -ManagedPath $ManagedPath -Restore
    }
}
$backup = Join-Path $ManagedPath 'Assembly-CSharp.dll.scen-trace-original'
$helperTarget = Join-Path $ManagedPath 'ScenRunner.RuntimeTrace.dll'
function Assert-KnownRestore([string]$BackupPath) {
    $backupHash = (Get-FileHash $BackupPath -Algorithm SHA256).Hash
    $currentHash = (Get-FileHash $target -Algorithm SHA256).Hash
    $known = @()
    $originals = @()
    foreach ($name in @('out','out_fix','out_contact_fix','out_collision_trace','out_collision_fix','out_spawn_trace','out_static_contact_fix','out_legacy_hashes')) {
        $built = Join-Path $ScenBuildRoot $name
        $assembly = Join-Path $built 'Assembly-CSharp.dll'
        if (Test-Path $assembly) { $known += (Get-FileHash $assembly -Algorithm SHA256).Hash }
        $history = Join-Path $built 'previous.sha256'
        if (Test-Path $history) { $known += @(Get-Content $history | ForEach-Object { $_.Trim() }) }
        $hashFile = Join-Path $built 'original.sha256'
        if (Test-Path $hashFile) { $originals += (Get-Content $hashFile).Trim() }
    }
    $sidecar = "$BackupPath.sha256"
    if (Test-Path $sidecar) {
        if ((Get-Content $sidecar).Trim() -ne $backupHash) { throw 'Backup hash mismatch. No files changed.' }
    } elseif (($known + $originals) -notcontains $backupHash) {
        throw 'Backup version cannot be verified. Build outputs or backup SHA256 are required.'
    }
    if (($known + $originals + @($backupHash)) -notcontains $currentHash) {
        throw 'Current DLL is unknown or updated. Refusing to restore an older backup.'
    }
}
$prior = Join-Path $ManagedPath 'Assembly-CSharp.dll.scen-before-collision-trace'
$priorHelper = Join-Path $ManagedPath 'ScenRunner.RuntimeTrace.dll.scen-before-collision-trace'
if (($LifecycleTrace -and $FixContacts) -or $RestoreContactFix) {
    $prior = Join-Path $ManagedPath 'Assembly-CSharp.dll.scen-before-collision-fix'
    $priorHelper = Join-Path $ManagedPath 'ScenRunner.RuntimeTrace.dll.scen-before-collision-fix'
}
if ($SpawnTrace -or $RestoreSpawnTrace) {
    $prior = Join-Path $ManagedPath 'Assembly-CSharp.dll.scen-before-spawn-trace'
    $priorHelper = Join-Path $ManagedPath 'ScenRunner.RuntimeTrace.dll.scen-before-spawn-trace'
}
if ($StaticContacts -or $RestoreStaticContacts) {
    $prior = Join-Path $ManagedPath 'Assembly-CSharp.dll.scen-before-static-contact-fix'
    $priorHelper = Join-Path $ManagedPath 'ScenRunner.RuntimeTrace.dll.scen-before-static-contact-fix'
}
if ($RestoreLifecycle -or $RestoreContactFix -or $RestoreSpawnTrace -or $RestoreStaticContacts) {
    if (!(Test-Path $prior)) { throw 'Pre-collision-trace backup not found.' }
    Assert-KnownRestore $prior
    if ((Test-Path "$priorHelper.sha256") -and
        (!(Test-Path $priorHelper) -or (Get-FileHash $priorHelper -Algorithm SHA256).Hash -ne (Get-Content "$priorHelper.sha256").Trim())) {
        throw 'Previous helper backup hash mismatch. No files changed.'
    }
    Copy-Item $prior $target -Force
    if (Test-Path $priorHelper) { Copy-Item $priorHelper $helperTarget -Force }
    elseif (Test-Path $helperTarget) { Remove-Item $helperTarget }
    Write-Output 'Previous DLLs restored. Restart MORAI.'; exit
}
if ($Restore) {
    if (!(Test-Path $backup)) { throw 'Original backup not found.' }
    Assert-KnownRestore $backup
    Copy-Item $backup $target -Force
    if (Test-Path $helperTarget) { Remove-Item $helperTarget }
    Write-Output 'Original DLL restored. All scen_runner DLL fixes and diagnostic hooks removed. Restart MORAI.'; exit
}
$folder = if ($StaticContacts) { 'out_static_contact_fix' } elseif ($SpawnTrace) { 'out_spawn_trace' } elseif ($LifecycleTrace -and $FixContacts) { 'out_collision_fix' } elseif ($LifecycleTrace) { 'out_collision_trace' } elseif ($FixContacts) { 'out_contact_fix' } elseif ($FixSelf) { 'out_fix' } else { 'out' }
$out = Join-Path $ScenBuildRoot $folder
if (!(Test-Path (Join-Path $out 'original.sha256'))) { throw 'Run build_trace.ps1 first.' }
$expected = (Get-Content (Join-Path $out 'original.sha256')).Trim()
$current = (Get-FileHash $target -Algorithm SHA256).Hash
$allowed = @($expected, (Get-FileHash (Join-Path $out 'Assembly-CSharp.dll') -Algorithm SHA256).Hash)
$oldTrace = Join-Path $ScenBuildRoot 'out/Assembly-CSharp.dll'
if ($FixSelf -and (Test-Path $oldTrace)) { $allowed += (Get-FileHash $oldTrace -Algorithm SHA256).Hash }
$previous = Join-Path $out 'previous.sha256'
if ($FixSelf -and (Test-Path $previous)) { $allowed += @(Get-Content $previous | ForEach-Object { $_.Trim() }) }
if ($FixContacts) {
    $oldFix = Join-Path $ScenBuildRoot 'out_fix/Assembly-CSharp.dll'
    if (Test-Path $oldFix) { $allowed += (Get-FileHash $oldFix -Algorithm SHA256).Hash }
    $oldHashes = Join-Path $ScenBuildRoot 'out_fix/previous.sha256'
    if (Test-Path $oldHashes) { $allowed += @(Get-Content $oldHashes | ForEach-Object { $_.Trim() }) }
}
if ($LifecycleTrace) {
    $oldFix = Join-Path $ScenBuildRoot 'out_fix/Assembly-CSharp.dll'
    if (Test-Path $oldFix) { $allowed += (Get-FileHash $oldFix -Algorithm SHA256).Hash }
    $oldHashes = Join-Path $ScenBuildRoot 'out_fix/previous.sha256'
    if (Test-Path $oldHashes) { $allowed += @(Get-Content $oldHashes | ForEach-Object { $_.Trim() }) }
    if (Test-Path $prior) { $allowed += (Get-FileHash $prior -Algorithm SHA256).Hash }
    $oldDiagnostic = Join-Path $ScenBuildRoot 'out_collision_trace/Assembly-CSharp.dll'
    if (Test-Path $oldDiagnostic) { $allowed += (Get-FileHash $oldDiagnostic -Algorithm SHA256).Hash }
    $oldDiagnosticHashes = Join-Path $ScenBuildRoot 'out_collision_trace/previous.sha256'
    if (Test-Path $oldDiagnosticHashes) { $allowed += @(Get-Content $oldDiagnosticHashes | ForEach-Object { $_.Trim() }) }
}
if ($SpawnTrace) {
    $oldCombined=Join-Path $ScenBuildRoot 'out_collision_fix/Assembly-CSharp.dll'
    if(Test-Path $oldCombined) {$allowed += (Get-FileHash $oldCombined -Algorithm SHA256).Hash}
    $combinedHistory=Join-Path $ScenBuildRoot 'out_collision_fix/previous.sha256'
    if(Test-Path $combinedHistory) {$allowed += @(Get-Content $combinedHistory | ForEach-Object {$_.Trim()})}
}
if ($StaticContacts) {
    # Accept only locally built variants of the same original simulator.
    foreach ($name in @('out_collision_fix','out_spawn_trace')) {
        $built = Join-Path $ScenBuildRoot $name
        $originalHash = Join-Path $built 'original.sha256'
        if (!(Test-Path $originalHash) -or (Get-Content $originalHash).Trim() -ne $expected) { continue }
        $assembly = Join-Path $built 'Assembly-CSharp.dll'
        if (Test-Path $assembly) { $allowed += (Get-FileHash $assembly -Algorithm SHA256).Hash }
        $history = Join-Path $built 'previous.sha256'
        if (Test-Path $history) { $allowed += @(Get-Content $history | ForEach-Object { $_.Trim() }) }
    }
}
if ($allowed -notcontains $current) { throw 'Unrecognized target DLL. Refusing to overwrite.' }
if (Test-Path $backup) {
    if ((Get-FileHash $backup -Algorithm SHA256).Hash -ne $expected) { throw 'Existing backup belongs to another version.' }
} else {
    if ($current -ne $expected) { throw 'Original backup required before replacing a modified DLL.' }
    Copy-Item $target $backup
}
$expected | Set-Content "$backup.sha256"
if (($LifecycleTrace -or $SpawnTrace -or $StaticContacts) -and !(Test-Path $prior)) {
    Copy-Item $target $prior
    (Get-FileHash $prior -Algorithm SHA256).Hash | Set-Content "$prior.sha256"
    if (Test-Path $helperTarget) {
        Copy-Item $helperTarget $priorHelper
        (Get-FileHash $priorHelper -Algorithm SHA256).Hash | Set-Content "$priorHelper.sha256"
    }
}
Copy-Item (Join-Path $out 'ScenRunner.RuntimeTrace.dll') $helperTarget -Force
Copy-Item (Join-Path $out 'Assembly-CSharp.dll') $target -Force
if ($StaticContacts) { Write-Output 'Static retirement contact cleanup installed. Existing self-contact/Pet1/vehicle/pedestrian fixes and spawn diagnostics retained. Restart MORAI. Live static collision/reuse validation required.' }
elseif ($SpawnTrace) { Write-Output 'Read-only native spawn trace installed. Existing self-contact/Pet1/contact fixes retained. Restart MORAI. No new spawn behavior fix.' }
elseif ($LifecycleTrace -and $FixContacts) { Write-Output 'Actor retirement contact references and mesh counters cleanup installed. Existing self-contact/Pet1 fixes retained. Restart MORAI; repeated-contact validation required.' }
elseif ($LifecycleTrace) { Write-Output 'Collision lifecycle diagnostic logs installed. Existing self-contact/Pet1 fixes retained; no new collision cleanup fix. Restart MORAI.' }
elseif ($FixContacts) { Write-Output 'Vehicle pool-retirement contact cleanup installed, with existing pedestrian fixes. Restart MORAI. Live collision validation is required.' }
elseif ($FixSelf) { Write-Output 'Self-contact guard and Ped_Pet1 PET behavior preservation installed. Physics remains enabled. Restart MORAI.' }
else { Write-Output 'Trace installed. Restart MORAI. Inspect [SCEN_RUNTIME_TRACE] in Player.log.' }
