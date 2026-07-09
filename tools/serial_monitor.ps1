param(
    [string]$Port = "",
    [int]$Baud = 115200
)

if (-not $Port) {
    $dev = Get-CimInstance -ClassName Win32_PnPEntity |
        Where-Object { $_.Name -match "STMicroelectronics STLink Virtual COM Port \((COM\d+)\)" } |
        Select-Object -First 1
    if ($dev -and ($dev.Name -match "(COM\d+)")) {
        $Port = $Matches[1]
    } else {
        Write-Host "Could not auto-detect the ST-Link Virtual COM Port. Pass -Port COMx explicitly." -ForegroundColor Red
        Write-Host "Available COM ports:" -ForegroundColor Yellow
        Get-CimInstance -ClassName Win32_PnPEntity | Where-Object { $_.Name -match "COM\d+" } | ForEach-Object { Write-Host "  $($_.Name)" }
        exit 1
    }
}

Write-Host "Opening $Port at $Baud baud (Ctrl+C to stop). Type a line + Enter to send it to the board." -ForegroundColor Cyan

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$sp.WriteTimeout = 2000
$sp.Encoding = [System.Text.Encoding]::ASCII

try {
    $sp.Open()
} catch {
    Write-Host "Failed to open $Port : $($_.Exception.Message)" -ForegroundColor Red
    Write-Host "Is another program (a debugger's semihosting view, another terminal) already using it?" -ForegroundColor Yellow
    exit 1
}

$outBuf = New-Object System.Text.StringBuilder
$inBuf  = New-Object System.Text.StringBuilder

try {
    while ($true) {
        # Drain anything the board sent, printing complete lines as they arrive.
        if ($sp.BytesToRead -gt 0) {
            $chunk = $sp.ReadExisting()
            foreach ($ch in $chunk.ToCharArray()) {
                if ($ch -eq "`n") {
                    Write-Host ($outBuf.ToString().TrimEnd("`r"))
                    [void]$outBuf.Clear()
                } else {
                    [void]$outBuf.Append($ch)
                }
            }
        }

        # Forward keystrokes to the board once a full line is typed.
        if ([Console]::KeyAvailable) {
            $key = [Console]::ReadKey($true)
            if ($key.Key -eq 'Enter') {
                $line = $inBuf.ToString()
                Write-Host ">> $line"
                $sp.Write("$line`r`n")
                [void]$inBuf.Clear()
            } elseif ($key.Key -eq 'Backspace') {
                if ($inBuf.Length -gt 0) { $inBuf.Length = $inBuf.Length - 1 }
            } else {
                [void]$inBuf.Append($key.KeyChar)
            }
        }

        Start-Sleep -Milliseconds 20
    }
} finally {
    if ($sp.IsOpen) { $sp.Close() }
}
