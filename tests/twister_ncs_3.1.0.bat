IF "%~1"=="" (
    echo Error: No COM port specified.
    exit /b 1
)
rmdir /s /q twister-out
SET PATH=C:\ncs\toolchains\b8b84efebd\opt\bin;C:\ncs\toolchains\b8b84efebd\opt\bin\Scripts;%PATH%
python C:\ncs\v3.1.0\zephyr\scripts\twister -T memlease_test -p nrf52840dk/nrf52840 --device-testing --device-serial %1
