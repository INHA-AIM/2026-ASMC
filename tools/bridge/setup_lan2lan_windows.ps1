[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('Simulator', 'Autonav')][string]$Role,
    [int]$InterfaceIndex = 0,
    [string]$ExpectedMacAddress = '',
    [switch]$Apply,
    [switch]$ConfigureWsl,
    [string]$StateDirectory = ''
)

$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)

function Set-IniValue([string]$Text, [string]$Section, [string]$Key, [string]$Value) {
    $lines = [System.Collections.Generic.List[string]]::new()
    foreach ($line in ($Text -split '\r?\n')) { $lines.Add($line) }
    $starts = @()
    for ($i = 0; $i -lt $lines.Count; $i++) {
        if ($lines[$i] -match ('^[ \t]*\[' + [regex]::Escape($Section) + '\][ \t]*([#;].*)?$')) { $starts += $i }
    }
    if ($starts.Count -gt 1) { throw "Duplicate [$Section] sections; review .wslconfig manually." }
    if ($starts.Count -eq 0) {
        if ($lines.Count -gt 0 -and $lines[$lines.Count - 1] -ne '') { $lines.Add('') }
        $lines.Add("[$Section]")
        $lines.Add("$Key=$Value")
    } else {
        $start = $starts[0]
        $end = $lines.Count
        for ($i = $start + 1; $i -lt $lines.Count; $i++) {
            if ($lines[$i] -match '^[ \t]*\[') { $end = $i; break }
        }
        $keyIndexes = @()
        for ($i = $start + 1; $i -lt $end; $i++) {
            if ($lines[$i] -match ('^[ \t]*' + [regex]::Escape($Key) + '[ \t]*=')) { $keyIndexes += $i }
        }
        if ($keyIndexes.Count -gt 1) { throw "Duplicate $Key entries; review .wslconfig manually." }
        if ($keyIndexes.Count -eq 1) { $lines[$keyIndexes[0]] = "$Key=$Value" }
        else { $lines.Insert($end, "$Key=$Value") }
    }
    return (($lines -join "`r`n").TrimEnd([char[]]"`r`n") + "`r`n")
}

function Resolve-LanAdapter([int]$Index, [string]$ExpectedMac, [string]$LocalIp) {
    if ($Index -eq 0 -and $ExpectedMac) {
        $candidates = @(Get-NetAdapter -Physical | Where-Object { $_.MacAddress -eq $ExpectedMac })
        if ($candidates.Count -ne 1) { throw 'Expected MAC does not identify exactly one physical adapter.' }
        $Index = $candidates[0].ifIndex
    } elseif ($Index -eq 0) {
        $indices = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
            Where-Object { $_.IPAddress -eq $LocalIp } | Select-Object -ExpandProperty InterfaceIndex -Unique)
        if ($indices.Count -ne 1) { throw 'Specify the dedicated adapter with -ExpectedMacAddress or -InterfaceIndex.' }
        $Index = $indices[0]
    }
    $value = Get-NetAdapter -InterfaceIndex $Index
    if ($ExpectedMac -and $value.MacAddress -ne $ExpectedMac) { throw 'Adapter MAC changed; no changes made.' }
    if ($value.Status -ne 'Up') { throw 'The selected cable adapter is not Up.' }
    return $value
}

function Assert-OwnedFirewallRule($Rule) {
    if ($Rule -and ($Rule.Group -ne 'ASMC LAN2LAN' -or [string]$Rule.Action -ne 'Allow')) {
        throw 'A named ASMC rule has unexpected ownership/action; no changes made.'
    }
}

function Set-AsmcWindowsRule([string]$Name, [hashtable]$Parameters) {
    $existing = Get-NetFirewallRule -Name $Name -ErrorAction SilentlyContinue
    Assert-OwnedFirewallRule $existing
    if ($existing) {
        Set-NetFirewallRule -Name $Name @Parameters | Out-Null
        return 'updated'
    }
    New-NetFirewallRule -Name $Name -DisplayName $Name -Group 'ASMC LAN2LAN' @Parameters | Out-Null
    return 'created'
}

function Set-AsmcHyperVRule([string]$Name, [string]$Creator, [hashtable]$Parameters) {
    $existing = Get-NetFirewallHyperVRule -Name $Name -ErrorAction SilentlyContinue
    if ($existing) {
        if ([guid]$existing.VMCreatorId -ne [guid]$Creator -or [string]$existing.Action -ne 'Allow') {
            throw 'A named Hyper-V rule has unexpected creator/action; no changes made.'
        }
        Set-NetFirewallHyperVRule -Name $Name @Parameters | Out-Null
        return 'updated'
    }
    New-NetFirewallHyperVRule -Name $Name -DisplayName $Name -VMCreatorId $Creator @Parameters | Out-Null
    return 'created'
}

if ($ConfigureWsl -and $Role -ne 'Autonav') { throw 'WSL changes belong on the Autonav PC only.' }
$localIp = if ($Role -eq 'Simulator') { '192.168.0.1' } else { '192.168.0.10' }
$peerIp = if ($Role -eq 'Simulator') { '192.168.0.10' } else { '192.168.0.1' }
$sensorPorts = @('9099', '9092', '9291', '9293', '9295', '9297', '9299', '9301', '9303')
$inPorts = if ($Role -eq 'Simulator') { @('9093') } else { $sensorPorts }
$outPorts = if ($Role -eq 'Simulator') { $sensorPorts } else { @('9093') }

$adapter = Resolve-LanAdapter $InterfaceIndex $ExpectedMacAddress $localIp
$InterfaceIndex = $adapter.ifIndex
$addresses = @(Get-NetIPAddress -InterfaceIndex $InterfaceIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue)
$foreign = @($addresses | Where-Object { $_.IPAddress -ne $localIp -and $_.IPAddress -notlike '169.254.*' })
if ($foreign.Count) { throw 'The selected NIC has another IPv4 address. Review it before changing it.' }
$duplicate = @(Get-NetIPAddress -AddressFamily IPv4 | Where-Object {
    $_.InterfaceIndex -ne $InterfaceIndex -and $_.IPAddress -like '192.168.0.*'
})
if ($duplicate.Count) { throw 'Another NIC already uses 192.168.0.x. Resolve the route conflict first.' }
$defaultOnCable = @(Get-NetRoute -InterfaceIndex $InterfaceIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object { $_.DestinationPrefix -eq '0.0.0.0/0' })
if ($defaultOnCable.Count) { throw 'The selected NIC has a default gateway. Refusing to interrupt an Internet adapter.' }
$existingTarget = @($addresses | Where-Object { $_.IPAddress -eq $localIp })
if (@($existingTarget | Where-Object { $_.PrefixLength -ne 24 }).Count) { throw 'Existing target address has a different subnet mask.' }

$wslPath = Join-Path $env:USERPROFILE '.wslconfig'
$wslBefore = if (Test-Path -LiteralPath $wslPath) { [IO.File]::ReadAllText($wslPath) } else { '' }
$wslAfter = $wslBefore
if ($ConfigureWsl) {
    if ([int](Get-CimInstance Win32_OperatingSystem).BuildNumber -lt 22621) { throw 'Mirrored networking requires Windows 11 22H2 or later.' }
    $wslAfter = Set-IniValue $wslAfter 'wsl2' 'networkingMode' 'mirrored'
    $wslAfter = Set-IniValue $wslAfter 'wsl2' 'firewall' 'true'
    if (-not (Get-Command Set-NetFirewallHyperVRule -ErrorAction SilentlyContinue)) { throw 'Update Windows/WSL before configuring the Hyper-V firewall.' }
}
$ruleNames = @("ASMC-LAN2LAN-$Role-UDP-Inbound-v1", "ASMC-LAN2LAN-$Role-UDP-Outbound-v1", "ASMC-LAN2LAN-$Role-Ping-Inbound-v1")
$rulesBefore = @(foreach ($name in $ruleNames) { Get-NetFirewallRule -Name $name -ErrorAction SilentlyContinue })
foreach ($rule in $rulesBefore) { Assert-OwnedFirewallRule $rule }
$creator = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
$hyperName = 'ASMC-LAN2LAN-Autonav-WSL-UDP-Inbound-v1'
$hyperBefore = $null
if ($ConfigureWsl) {
    $hyperBefore = Get-NetFirewallHyperVRule -Name $hyperName -ErrorAction SilentlyContinue
    if ($hyperBefore -and ([guid]$hyperBefore.VMCreatorId -ne [guid]$creator -or [string]$hyperBefore.Action -ne 'Allow')) {
        throw 'Existing Hyper-V rule is not the expected WSL allow rule; no changes made.'
    }
}
$plan = [ordered]@{
    Role = $Role; Adapter = $adapter.Name; InterfaceIndex = $InterfaceIndex
    MacAddress = $adapter.MacAddress; LinkSpeed = [string]$adapter.LinkSpeed
    LocalAddress = "$localIp/24"; PeerAddress = $peerIp
    InboundUdp = $inPorts; OutboundUdp = $outPorts
    ExistingIpv4 = @($addresses | Select-Object IPAddress, PrefixLength)
    ConfigureWsl = [bool]$ConfigureWsl; Apply = [bool]$Apply
    RestartWsl = $false; RestartDocker = $false
}
if (-not $Apply) { $plan | ConvertTo-Json -Depth 5; return }
$admin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole(
    [Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $admin) { throw 'Run -Apply from Administrator PowerShell. No changes made.' }
if (-not $StateDirectory) { $StateDirectory = Join-Path $env:LOCALAPPDATA 'ASMC\lan2lan' }
New-Item -ItemType Directory -Path $StateDirectory -Force | Out-Null
$statePath = Join-Path $StateDirectory ('state-' + [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss-ffff') + '.json')
$ipInterface = Get-NetIPInterface -InterfaceIndex $InterfaceIndex -AddressFamily IPv4
$state = [ordered]@{
    Plan = $plan; StartedUtc = [DateTime]::UtcNow.ToString('o'); Status = 'applying'
    Before = [ordered]@{
        Dhcp = [string]$ipInterface.Dhcp
        Ipv4 = @($addresses | Select-Object IPAddress, PrefixLength, PrefixOrigin)
        Dns = @((Get-DnsClientServerAddress -InterfaceIndex $InterfaceIndex -AddressFamily IPv4).ServerAddresses)
        DefaultRoutes = @(Get-NetRoute -AddressFamily IPv4 | Where-Object { $_.DestinationPrefix -eq '0.0.0.0/0' } |
            Select-Object InterfaceIndex, DestinationPrefix, NextHop, RouteMetric)
    }
    AddedIp = $false; RulesCreated = @(); RulesUpdated = @(); HyperVRulesCreated = @(); HyperVRulesUpdated = @()
    WslChanged = $false; WslBackup = ''; Error = ''
}
function Save-State { $state | ConvertTo-Json -Depth 9 | Set-Content -LiteralPath $statePath -Encoding UTF8 }
Save-State
[ordered]@{
    Rules = $rulesBefore
    PortFilters = @($rulesBefore | Get-NetFirewallPortFilter)
    AddressFilters = @($rulesBefore | Get-NetFirewallAddressFilter)
    InterfaceFilters = @($rulesBefore | Get-NetFirewallInterfaceFilter)
    HyperVRule = $hyperBefore
} | Export-Clixml -LiteralPath ($statePath + '.firewall-before.xml')
try {
    if (-not $existingTarget.Count) {
        New-NetIPAddress -InterfaceIndex $InterfaceIndex -AddressFamily IPv4 -IPAddress $localIp -PrefixLength 24 | Out-Null
        $state.AddedIp = $true
        Save-State
    }
    # Only the dedicated cable NIC is touched; do not add a gateway or DNS server.
    Set-DnsClientServerAddress -InterfaceIndex $InterfaceIndex -ResetServerAddresses
    $common = @{
        Action = 'Allow'; Enabled = 'True'; Profile = 'Any'
        InterfaceAlias = $adapter.Name; LocalAddress = $localIp; RemoteAddress = $peerIp
    }
    foreach ($direction in @('Inbound', 'Outbound')) {
        $name = "ASMC-LAN2LAN-$Role-UDP-$direction-v1"
        $parameters = $common.Clone()
        $parameters.Direction = $direction; $parameters.Protocol = 'UDP'
        $parameters.LocalPort = if ($direction -eq 'Inbound') { $inPorts } else { 'Any' }
        $parameters.RemotePort = if ($direction -eq 'Outbound') { $outPorts } else { 'Any' }
        $operation = Set-AsmcWindowsRule $name $parameters
        if ($operation -eq 'created') { $state.RulesCreated += $name } else { $state.RulesUpdated += $name }
        Save-State
    }
    $name = "ASMC-LAN2LAN-$Role-Ping-Inbound-v1"
    $parameters = $common.Clone(); $parameters.Direction = 'Inbound'; $parameters.Protocol = 'ICMPv4'; $parameters.IcmpType = 8
    $operation = Set-AsmcWindowsRule $name $parameters
    if ($operation -eq 'created') { $state.RulesCreated += $name } else { $state.RulesUpdated += $name }
    Save-State
    if ($ConfigureWsl) {
        $parameters = @{Direction='Inbound'; Action='Allow'; Protocol='UDP'; LocalAddresses=$localIp; RemoteAddresses=$peerIp; LocalPorts=$inPorts; RemotePorts='Any'}
        $operation = Set-AsmcHyperVRule $hyperName $creator $parameters
        if ($operation -eq 'created') { $state.HyperVRulesCreated += $hyperName } else { $state.HyperVRulesUpdated += $hyperName }
        Save-State
        if ($wslAfter -ne $wslBefore) {
            if (Test-Path -LiteralPath $wslPath) {
                $state.WslBackup = Join-Path $StateDirectory ((Split-Path $statePath -Leaf) + '.wslconfig-before')
                Copy-Item -LiteralPath $wslPath -Destination $state.WslBackup
                Save-State
            }
            [IO.File]::WriteAllText($wslPath, $wslAfter, [Text.UTF8Encoding]::new($false))
            $state.WslChanged = $true
            Save-State
        }
    }
    $state.Status = 'configured'
    $state['After'] = @((Get-NetIPAddress -InterfaceIndex $InterfaceIndex -AddressFamily IPv4) |
        Select-Object IPAddress, PrefixLength, AddressState)
    $state['FinishedUtc'] = [DateTime]::UtcNow.ToString('o')
    Save-State
    Write-Output "CONFIGURED $Role $localIp/24 on $($adapter.Name). State: $statePath"
    if ($ConfigureWsl) { Write-Output 'Restart WSL/Docker after saving work; enable Docker Desktop host networking. This script does not restart them.' }
} catch {
    $state.Status = 'failed'
    $state.Error = $_.Exception.Message
    Save-State
    throw "Setup failed; inspect partial changes in ${statePath}: $($state.Error)"
}
