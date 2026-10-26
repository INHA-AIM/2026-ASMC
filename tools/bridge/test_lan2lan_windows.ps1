$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
$scriptPath = Join-Path $PSScriptRoot 'setup_lan2lan_windows.ps1'
$tokens = $null; $errors = $null
$ast = [System.Management.Automation.Language.Parser]::ParseFile($scriptPath, [ref]$tokens, [ref]$errors)
if ($errors.Count) { throw ($errors | Out-String) }
foreach ($command in @('Set-NetFirewallRule', 'Set-NetFirewallHyperVRule')) {
    $metadata = Get-Command $command
    $required = if ($command -eq 'Set-NetFirewallRule') {
        @('InterfaceAlias', 'LocalAddress', 'RemoteAddress', 'Protocol', 'LocalPort', 'RemotePort', 'Direction', 'IcmpType')
    } else { @('LocalAddresses', 'RemoteAddresses', 'Protocol', 'LocalPorts', 'RemotePorts', 'Direction') }
    foreach ($name in $required) { if (-not $metadata.Parameters.ContainsKey($name)) { throw "Unsupported native parameter: $command $name" } }
}
$ast.FindAll({param($n) $n -is [System.Management.Automation.Language.FunctionDefinitionAst]}, $true) |
    ForEach-Object { Invoke-Expression $_.Extent.Text }

function Assert($condition, [string]$message) { if (-not $condition) { throw $message } }
function Assert-Throws([scriptblock]$Body) {
    $threw = $false
    try { & $Body | Out-Null } catch { $threw = $true }
    Assert $threw 'Expected preflight rejection did not occur.'
}
$creator = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
$global:AsmcLanTestCalls = @()
$global:AsmcLanTestRule = [pscustomobject]@{Group='ASMC LAN2LAN'; Action='Allow'}
$global:AsmcLanTestHyper = [pscustomobject]@{VMCreatorId=([guid]$creator); Action='Allow'}
$global:AsmcLanTestUsbUp = $true
$global:AsmcLanTestConflict = $false
function Get-NetAdapter([int]$InterfaceIndex, [switch]$Physical) {
    $items = @(
        [pscustomobject]@{ifIndex=17;Name='Builtin';MacAddress='10-7C-61-43-AB-C0';Status='Up';LinkSpeed='1 Gbps'},
        [pscustomobject]@{ifIndex=71;Name='USB';MacAddress='70-5D-CC-F8-39-14';Status=$(if($global:AsmcLanTestUsbUp){'Up'}else{'Disconnected'});LinkSpeed='1 Gbps'})
    if ($Physical) { return $items }
    return $items | Where-Object { $_.ifIndex -eq $InterfaceIndex }
}
function Get-NetIPAddress([int]$InterfaceIndex, [string]$AddressFamily, [string]$ErrorAction) {
    return [pscustomobject]@{IPAddress='192.168.0.10';InterfaceIndex=$(if($global:AsmcLanTestConflict){17}else{71});PrefixLength=24}
}
function Get-NetRoute([int]$InterfaceIndex, [string]$AddressFamily, [string]$ErrorAction) { return @() }
function Get-NetFirewallRule([string]$Name, [string]$ErrorAction) { return $global:AsmcLanTestRule }
function Set-NetFirewallRule { $global:AsmcLanTestCalls += ,@($args); }
function New-NetFirewallRule { $global:AsmcLanTestCalls += ,@('create', $args); }
function Get-NetFirewallHyperVRule([string]$Name, [string]$ErrorAction) { return $global:AsmcLanTestHyper }
function Set-NetFirewallHyperVRule { $global:AsmcLanTestCalls += ,@($args); }
function New-NetFirewallHyperVRule { $global:AsmcLanTestCalls += ,@('create-hyper', $args); }

$usb = Resolve-LanAdapter 0 '70-5D-CC-F8-39-14' '192.168.0.10'
Assert ($usb.ifIndex -eq 71) 'MAC-first lookup did not follow a USB port/index change.'
Assert-Throws { Resolve-LanAdapter 17 '70-5D-CC-F8-39-14' '192.168.0.10' }
$global:AsmcLanTestUsbUp = $false
Assert-Throws { Resolve-LanAdapter 0 '70-5D-CC-F8-39-14' '192.168.0.10' }
$global:AsmcLanTestUsbUp = $true
$parameters = @{InterfaceAlias='USB';LocalPort=@('9099','9303');RemotePort='Any';Direction='Inbound';Protocol='UDP';LocalAddress='192.168.0.10';RemoteAddress='192.168.0.1'}
Assert ((Set-AsmcWindowsRule 'ASMC-test' $parameters) -eq 'updated') 'Existing rule must update, not silently skip.'
Assert (($global:AsmcLanTestCalls[-1] -contains 'USB') -and ($global:AsmcLanTestCalls[-1] -contains '192.168.0.1')) 'Scoped rule parameters were not passed to the setter.'
$before = $global:AsmcLanTestCalls.Count
$global:AsmcLanTestRule = [pscustomobject]@{Group='Unrelated';Action='Allow'}
Assert-Throws { Set-AsmcWindowsRule 'ASMC-test' $parameters }
Assert ($global:AsmcLanTestCalls.Count -eq $before) 'An unrelated rule was changed.'
$global:AsmcLanTestRule = $null
Assert ((Set-AsmcWindowsRule 'ASMC-test' $parameters) -eq 'created') 'Missing rule must be created.'
Assert ((Set-AsmcHyperVRule 'ASMC-hyper' $creator @{LocalPorts=@('9099','9303');RemotePorts='Any'}) -eq 'updated') 'Existing Hyper-V rule must update; GUID normalization must be accepted.'
$global:AsmcLanTestRule = [pscustomobject]@{Group='ASMC LAN2LAN';Action='Allow'}
$before = $global:AsmcLanTestCalls.Count
$preview = & $scriptPath -Role Autonav -ExpectedMacAddress '70-5D-CC-F8-39-14' | ConvertFrom-Json
Assert ($global:AsmcLanTestCalls.Count -eq $before) 'Dry run modified firewall rules.'
Assert ($preview.InterfaceIndex -eq 71 -and $preview.InboundUdp -contains '9099' -and $preview.InboundUdp -notcontains '9798') 'Dry-run plan has the wrong NIC or status ports.'
$global:AsmcLanTestConflict = $true
Assert-Throws { & $scriptPath -Role Autonav -ExpectedMacAddress '70-5D-CC-F8-39-14' }
Assert ($global:AsmcLanTestCalls.Count -eq $before) 'IP conflict caused mutation.'
$ini = "[wsl2]`r`nmemory=12GB`r`nprocessors=8`r`nnetworkingMode=nat`r`n[experimental]`r`nautoMemoryReclaim=gradual`r`n"
$merged = Set-IniValue $ini 'wsl2' 'networkingMode' 'mirrored'
$merged = Set-IniValue $merged 'wsl2' 'firewall' 'true'
Assert ($merged.Contains('memory=12GB') -and $merged.Contains('processors=8') -and $merged.Contains('autoMemoryReclaim=gradual')) 'Unrelated WSL settings were lost.'
Assert ($merged.Contains('networkingMode=mirrored') -and $merged.Contains('firewall=true')) 'Expected WSL values are missing.'
Assert-Throws { Set-IniValue "[wsl2]`n[wsl2]" 'wsl2' 'firewall' 'true' }
Write-Output 'PASS: native parameters, MAC resolution, wrong/down NIC, owned rule update/create, unrelated rule rejection, Hyper-V update, dry run, IP conflict, INI preservation.'
