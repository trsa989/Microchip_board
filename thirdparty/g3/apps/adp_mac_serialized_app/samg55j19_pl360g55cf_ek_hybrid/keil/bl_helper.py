import sys
import os
#import numpy as np
from datetime import datetime
from shutil import copy2
import subprocess
import glob


# CRC stuff ************
CRC16_POLY = 0x1021
def add_crc(data, crc):
    crc = crc ^ (data << 8)

    for cmpt in range(8):
        if crc & 0x8000:
            crc = (crc << 1) ^ CRC16_POLY
        else:
            crc = crc << 1

    return crc & 0xFFFF


def eval_crc16(data, length):
    crc = 0

    for cpt in range(length):
        c = data[cpt]
        crc = add_crc(c, crc)

    return crc
# ***********************

#test code



#code starts here

print("Files needed: ", sys.argv[1:])

adp_mac_file = sys.argv[1]
if not os.path.exists(adp_mac_file):
    print("AdpMacApp file doesnt exist ./: " + adp_mac_file)
    exit()

bootloader_file = r"..\..\..\..\..\..\sam\applications\serial_bootloader\samg55j19_pl360g55cx_ek\as5_arm" + "\\" +sys.argv[2]
if not os.path.exists(bootloader_file):
    print("Bootloader file doesnt exist: " + bootloader_file)
    exit()
    
print("Both bin files found")


#check if sign tool exist
sign_tool_path = r"..\..\..\..\..\..\FirmwareManagementToolCli\FirmwareManagementToolCli.exe"
if not os.path.exists(sign_tool_path):
    print("Sign tool doesnt exist at: " + sign_tool_path)
    exit()

print("Sign tool located at: " + sign_tool_path)


boot_bin = None
with open(bootloader_file, 'r+b') as f:
    boot_bin = f.read()


print("Boot file size:", len(boot_bin))

with open(adp_mac_file, 'r+b') as f:
    # Write bin file size
    file_stats = os.stat(adp_mac_file)
    file_size = file_stats.st_size
    #insert app file size
    f.seek(0x1E0)
    f.write(file_size.to_bytes(4, 'little'))
    #insert boot file size
    f.seek(0x1E4)
    f.write(len(boot_bin).to_bytes(4, 'little'))
    
    f.seek(0)
    all_but_2_bytes = f.read()[:-2]
    res_crc = eval_crc16(all_but_2_bytes, len(all_but_2_bytes))
    print("CRC BYTES: ", res_crc)
    
    f.seek(-2, os.SEEK_END)
    f.write(res_crc.to_bytes(2, 'little'))
    print("Appending bootloader bin")
    #append boot bin and insert boot bin size
    f.write(boot_bin)
    

# copy and rename file
timestamp = str(datetime.now())[:16]
timestamp = timestamp.replace(':', '_')
timestamp = timestamp.replace(' ', '_')
timestamp = timestamp.replace('-', '_')
src_path = adp_mac_file
new_file_path = f'{src_path[:-4]}_{timestamp}.bin'
#new_file_path = f'example-{timestamp}.txt' # good formatting example
copy2(src_path, new_file_path)
    

#write size in file
file_stats = os.stat(adp_mac_file)
file_size = hex(file_stats.st_size).upper()
newText= f"S20010000,{file_size[2:]}#"
#print(newText)
file=open("bininfo.txt","w")
file.write(newText)
file.close()


#sign bin file     
# FirmwareManagementToolCli.exe  -s -i "apps_template_app.bin" -e ISKMO5 -f ISK-IMG-MOD-ISK -g 04
exe_path = sign_tool_path
arg1 = "-s"
arg2 = "-i"
arg3 = new_file_path
arg4 = "-e"
arg5 = "ISKMO5"
arg6 = "-f"
arg7 = "ISK-IMG-MOD-ISK"
arg8 = "-g"
arg9 = "04"

try:
    subprocess.run([exe_path, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9], check=True)
except subprocess.CalledProcessError as e:
    print(f"Error: {e}")

#delete unused files
print("Deleting unused files...")
fileList = glob.glob('./*DevelopmentKey.mvfw')
fileList += glob.glob('./*DevelopmentKey.xml') 
fileList += glob.glob('./*DevelopmentKey.txt') 
for filePath in fileList:
    try:
        os.remove(filePath)
    except:
        print("Error while deleting file : ", filePath)
        
