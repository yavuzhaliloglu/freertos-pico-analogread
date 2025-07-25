import os
import subprocess

# Build klasörü oluşturulmamışsa oluştur
build_dir = "build"
if not os.path.exists(build_dir):
    os.makedirs(build_dir)

# CMake komutunu build dizininde çalıştır
result = subprocess.run(
    ["cmake", "..", "-DPICO_COPY_TO_RAM=1"],
    cwd=build_dir,
    capture_output=True,
    text=True
)
print("CMake output:")
print(result.stdout)

# Make komutunu build dizininde çalıştır
result = subprocess.run(
    ["make", "-j4"],
    cwd=build_dir,
    capture_output=True,
    text=True
)
print("Make output:")
print(result.stdout)

# blink.uf2 dosyasının build klasöründe olduğunu varsay
uf2_path = os.path.join(build_dir, "blink.uf2")
result = subprocess.run(
    ["sudo", "picotool", "load", uf2_path, "-F"],
    capture_output=True,
    text=True
)
print("Picotool output:")
print(result.stdout)