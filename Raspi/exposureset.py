import socket
import os
from paramiko import SSHClient, AutoAddPolicy
import tkinter as tk
from PIL import ImageTk, Image


pi_IP = '10.6.70.26'
pi_USERNAME='pi'
pi_PASSWORD='hhs670'
vision_folder_path = '/home/pi/Vision/'
current_path=os.path.dirname(os.path.realpath(__file__))
exposure = .1


def Connect(ip, username=pi_USERNAME, pw=pi_PASSWORD): 
    '''ssh into the pi'''
    print('connecting to {}@{}...'.format(username, ip))
    ssh = SSHClient()
    ssh.set_missing_host_key_policy(AutoAddPolicy())
    ssh.connect(ip, username=username, password=pw)
    print('connection status =', ssh.get_transport().is_active())
    return ssh

def SendCommand(ssh, command, pw='password'):
    '''send a terminal/bash command to the ssh'ed-into machine '''
    print('sending a command... ', command)
    stdin, stdout, stderr = ssh.exec_command( command )
    if "sudo" in command:
        stdin.write(pw+'\n')
    stdin.flush()
    print('\nstout:',stdout.read())
    print('\nsterr:',stderr.read())

def Update():
    host_name = socket.gethostname()
    host_ip = socket.gethostbyname(host_name)
    username = os.path.split(os.path.expanduser('~'))[-1]
    SendCommand(myssh, command='v4l2-ctl -d /dev/video0 -c exposure_auto=1 -c exposure_absolute='+str(exposure)+' -c brightness=0 -c white_balance_temperature_auto=0 -c backlight_compensation=0 -c contrast=10 -c saturation=200')
    SendCommand(myssh, command='python '+vision_folder_path+'exposure.py')
    SendCommand(myssh, command='scp '+vision_folder_path+'exposuretest.jpg '+str(username)+'@'+str(host_ip)+':'+current_path.replace('\\', '/'))
    SendCommand(myssh, command='rm '+vision_folder_path+'exposuretest.jpg')
    print("Updated!")


def UpdateButtonClicked():
    Update()
    print(exposure)
    img = ImageTk.PhotoImage(file=current_path+"\exposuretest.jpg")
    label.configure(image = img)
    label.photo = img




myssh = Connect(ip=pi_IP)
Update()


root = tk.Tk()      

img = ImageTk.PhotoImage(file=current_path+"\exposuretest.jpg")  
label = tk.Label(root, image = img)
label.pack()

button = tk.Button(root, text = "Update", command=UpdateButtonClicked)
button.pack()

def testVal(inStr,acttyp):
    return
    if acttyp == '1': #insert
        if not inStr.isdigit():
            return False
    return True

label2 = tk.Label(root, text="Exposure")
label2.pack(side = "left")
entry = tk.Entry(root, validate="key")
entry['validatecommand'] = (entry.register(testVal),'%P','%d')
entry.pack(fill = tk.X)
    

tk.mainloop()













