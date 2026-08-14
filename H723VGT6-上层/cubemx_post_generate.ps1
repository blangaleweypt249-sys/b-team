$ErrorActionPreference = 'Stop'

$projectRoot = $PSScriptRoot
$projectFile = Join-Path $projectRoot 'MDK-ARM\H723VGT6.uvprojx'
$portableRoot = Join-Path $projectRoot 'Middlewares\Third_Party\FreeRTOS\Source\portable'

if (-not (Test-Path -LiteralPath $projectFile)) {
    exit 0
}

$text = [IO.File]::ReadAllText($projectFile)
$freertosFile = Join-Path $projectRoot 'Core\Src\freertos.c'
$gccInclude = '../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1'
$cmsisRtos2Include = '../Drivers/CMSIS/RTOS2/Include'
$rvdsInclude = '../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F'
$localGccInclude = '../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F'
$localGccM7Include = '../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1'
$obsoleteVofaInclude = '../User/Application/Vofa'

# Keep the include paths aligned with the actual source tree.
$text = $text.Replace(";$rvdsInclude", '')
$text = $text.Replace(";$localGccInclude", '')
$text = $text.Replace(";$localGccM7Include", '')
$text = $text.Replace(";$obsoleteVofaInclude", '')
$text = $text.Replace("$obsoleteVofaInclude;", '')

# Motor movement test/debug code is not part of the production target.
$vofaGroupPattern = '(?s)\s*<Group>\s*<GroupName>User/Application/Vofa</GroupName>.*?</Group>'
$text = [regex]::Replace($text, $vofaGroupPattern, '', 1)

# Remove any RVDS or locally generated ARM_CM4F/ARM_CM7 port entry.
$portPaths = @(
    '../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c',
    '../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c',
    '../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c'
)
foreach ($portPath in $portPaths) {
    $escapedPath = [regex]::Escape($portPath)
    $pattern = "(?s)\s*<File>\s*<FileName>port\.c</FileName>\s*<FileType>1</FileType>\s*<FilePath>$escapedPath</FilePath>.*?</File>"
    $text = [regex]::Replace($text, $pattern, '', 1)
}

# Keep the external GCC ARM_CM7 include directory on target C settings.
$includePattern = '(?s)<IncludePath>([^<]*FreeRTOS/Source/include[^<]*)</IncludePath>'
$text = [regex]::Replace($text, $includePattern, {
    param($match)
    $value = $match.Groups[1].Value
    $value = $value.Replace(";$rvdsInclude", '').Replace(";$localGccInclude", '').Replace(";$localGccM7Include", '').Replace(";$obsoleteVofaInclude", '').Replace("$obsoleteVofaInclude;", '')
    if ($value -notlike "*$gccInclude*") {
        $value += ";$gccInclude"
    }
    if ($value -notlike "*$cmsisRtos2Include*") {
        $value = "$cmsisRtos2Include;$value"
    }
    "<IncludePath>$value</IncludePath>"
})

# Add the single GCC ARM_CM7 port to the FreeRTOS group.
$gccPort = @'
            <File>
              <FileName>port.c</FileName>
              <FileType>1</FileType>
              <FilePath>../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c</FilePath>
            </File>
'@
$groupPattern = '(?s)(<Group>\s*<GroupName>Middlewares/FreeRTOS</GroupName>.*?<Files>.*?)(\s*</Files>)'
$text = [regex]::Replace($text, $groupPattern, ('$1' + $gccPort + '$2'), 1)

$text = $text.TrimEnd() + [Environment]::NewLine
[IO.File]::WriteAllText($projectFile, $text, [Text.UTF8Encoding]::new($false))

if (Test-Path -LiteralPath $freertosFile) {
    $freertosText = [IO.File]::ReadAllText($freertosFile)
    $freertosText = $freertosText.Replace(
        'void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);',
        'void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);')
    $freertosText = $freertosText.Replace(
        'vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)',
        'vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)')
    [IO.File]::WriteAllText($freertosFile, $freertosText, [Text.UTF8Encoding]::new($false))
}

foreach ($obsolete in @('RVDS', 'GCC\ARM_CM4F')) {
    $obsoletePath = [IO.Path]::GetFullPath((Join-Path $portableRoot $obsolete))
    $portableFull = [IO.Path]::GetFullPath($portableRoot)
    if ($obsoletePath.StartsWith($portableFull + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase) -and (Test-Path -LiteralPath $obsoletePath)) {
        Remove-Item -LiteralPath $obsoletePath -Recurse -Force
    }
}
