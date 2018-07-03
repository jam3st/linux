rm -rf stag/
mkdir -p stag/efi/boot
make -j12 &&
cp arch/x86/boot/bzImage stag/efi/boot/bootx64.efi &&
/usr/bin/qemu-system-x86_64 -m 2048 -L /mnt/work/vm/etc/ -no-reboot \
    -vga none -device vfio-pci,host=00:00:02.0,x-vga=off,addr=0x2,x-igd-opregion=on -net none \
    -smp 4,threads=2 -machine q35,accel=kvm,usb=off,vmport=off,kernel_irqchip=on,igd-passthru=on \
    -drive if=pflash,format=raw,readonly,file=/mnt/work/vm/etc/OVMF.fd -cpu host -display none -monitor none \
    -drive id=d0,if=none,file.driver=vvfat,readonly,file.dir=./stag \
    -device virtio-scsi-pci -device scsi-hd,drive=d0,bootindex=0 \
     -chardev stdio,id=qemudbg -device isa-debugcon,iobase=0x402,chardev=qemudbg \
    -serial file:stdio.log

cd cov 
find . -maxdepth 1 -type d | xargs rm -rf
./a.py | base64 -d | bzip2 -dc | tar -xf -
lcov -o kern.info -d . -c
genhtml kern.info
