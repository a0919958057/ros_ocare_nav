import paramiko
from time import localtime, strftime

# Create a ssh client
client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect('ocare', username='taiwanet', password='qwe7891')

file_name = strftime("/home/taiwanet/capture_image/%Y%m%d_%H%M%S.jpg", localtime())

stdin, stdout, stderr = client.exec_command('fswebcam --no-banner -d /dev/video1 brightness=112 -s Contrast=37 -s Gamma=50% -s Saturation=51 -s Sharpness=20 -p YUYV -S 5 -D 1 -r 1280x960 --jpeg 100 '+ file_name)

print('Capture Image \''+file_name+'\' successful! ')

for line in stderr:
    print(line)

for line in stdout:
    print(line)



client.close()
