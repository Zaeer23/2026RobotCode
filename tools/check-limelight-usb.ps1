# Checks whether a Limelight plugged into THIS computer over USB has enumerated.
#
# A Limelight on USB appears as a USB-ethernet gadget: the PC gets an address on 172.29.0.x and the
# camera answers on 172.29.0.1. If the camera has no 12V power it never boots, the gadget never
# enumerates, and Windows shows "Unknown USB Device (Device Descriptor Request Failed)" instead.
#
# Usage:  powershell -ExecutionPolicy Bypass -File tools\check-limelight-usb.ps1

Write-Host "=== Limelight USB check ===" -ForegroundColor Cyan

# 1. Did anything fail to enumerate? VID_0000 means Windows never read a valid descriptor,
#    which for a Limelight almost always means it is not powered.
$failed = Get-PnpDevice -Class USB -ErrorAction SilentlyContinue |
    Where-Object { $_.Present -and $_.Status -eq 'Error' }

if ($failed) {
    Write-Host "`n[!] USB device(s) failed to enumerate:" -ForegroundColor Red
    $failed | Format-Table -AutoSize FriendlyName, InstanceId
    Write-Host "    A Limelight cannot boot on USB power alone. Give it 12V through the PoE" -ForegroundColor Yellow
    Write-Host "    injector first, then reconnect USB. Also confirm the cable carries DATA," -ForegroundColor Yellow
    Write-Host "    not just charge." -ForegroundColor Yellow
}

# 2. Is there a USB-ethernet gadget interface at all?
$gadget = Get-PnpDevice -Class Net -ErrorAction SilentlyContinue |
    Where-Object { $_.Present -and $_.FriendlyName -match 'USB|RNDIS|Remote NDIS|Gadget' }

if ($gadget) {
    Write-Host "`n[ok] USB network adapter present:" -ForegroundColor Green
    $gadget | Format-Table -AutoSize FriendlyName, Status
} else {
    Write-Host "`n[!] No USB network adapter present - the camera has not enumerated." -ForegroundColor Red
}

# 3. Did we get an address on the Limelight's USB subnet?
$usbIp = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object { $_.IPAddress -like '172.29.*' }

if ($usbIp) {
    Write-Host "`n[ok] This PC has a Limelight-subnet address:" -ForegroundColor Green
    $usbIp | Format-Table -AutoSize InterfaceAlias, IPAddress

    Write-Host "`nTesting http://172.29.0.1:5801 ..." -ForegroundColor Cyan
    try {
        Invoke-WebRequest -Uri 'http://172.29.0.1:5801' -TimeoutSec 4 -UseBasicParsing | Out-Null
        Write-Host "[ok] Web UI responded. Open http://172.29.0.1:5801" -ForegroundColor Green
    } catch {
        Write-Host "[!] Interface is up but the camera did not answer on 5801." -ForegroundColor Red
    }
} else {
    Write-Host "`n[!] No 172.29.x.x address - nothing to talk to yet." -ForegroundColor Red
}

Write-Host "`n=== done ===" -ForegroundColor Cyan
