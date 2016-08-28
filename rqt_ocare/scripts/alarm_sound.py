import paramiko
from time import localtime, strftime

# Create a ssh client
client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect('ocare', username='taiwanet', password='qwe7891')

stdin, stdout, stderr = client.exec_command('aplay -D plughw:0,0 /home/taiwanet/6175.wav')

print('Alarm Now! ')

for line in stderr:
    print(line)

for line in stdout:
    print(line)



client.close()
