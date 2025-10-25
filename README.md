# expm_SSC

## build

First, generate code by ET9300 SSC with project at `Core/Src/SSC/SSC-Device.esp`.

Then, generate code by stm32cubeMX GUI, `STM32CubeMX expm_SSC.ioc`.

After code generation, then build

```bash
cmake --preset=Debug
cmake --build --preset=Debug
```

then flash binary to MCU (modify depend on your debugger and target)

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg -c "program build/Debug/expm_SSC.elf verify reset exit"
# openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/Debug/expm_SSC.elf verify reset exit"
```

### write SII

assume SSC code generated, for details of how to generate SSC code, refer to `Core/Src/SSC`.

```bash
siitool ./Core/Src/SSC/SSC-Device.xml -o ./build/Debug/SSC-Device.bin
```

assume [IgH ethercat master](https://gitlab.com/etherlab.org/ethercat) installed

```bash
sudo ethercat rescan && sudo ethercat sii_write ./build/Debug/SSC-Device.bin
```

### debug

```bash
sudo ethercat slaves
```

```bash
minicom -D /dev/ttyUSB0
```

`gdb` combined `openocd` is useful debug too, checkout `marus25.cortex-debug`.

If you want to check what will write in SII ESC Configuration Area, [esc-configdata-gen](https://github.com/existedinnettw/esc-configdata-gen) may help.

```bash
esc-configdata-gen -d $(grep -oP '(?<=<ConfigData>)[^<]+' ./Core/Src/SSC/SSC-Device.xml)
```

### bonus

You can run SSC under windows container, follow [dockur/windows](https://github.com/dockur/windows?tab=readme-ov-file)

```bash
docker compose up -d
xfreerdp3 /v:localhost /u:Docker
```

## ref

* `Application Note ET9300(EtherCAT Slave Stack Code)`
* `Ethercat Slave Controller Section II – Register Description`
* `Ethercat Slave Controller Section III – Hardware Description`
