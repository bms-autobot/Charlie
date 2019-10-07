# Automatically deploy updated code files to the working directory on Charlie
# Authored By: Connor Mulcahy

# Need to setup SCP for transferring files between systems with SSH

import paramiko

try:
    # Create new SSHClient for SSH interaction
    client = paramiko.client.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.WarningPolicy)

    client.connect(hostname = "141.219.20.99", port = 22, username = "ubuntu", password = "ubuntu")

    #client.exec_command('cd workspace')

    # Put us in our current working directory, list contents including hidden
    stdin, stdout, stderr = client.exec_command('cd workspace;ls -l')

    # Dump all stored lines in 
    for line in stdout.read().decode().splitlines():
        print(line)


except Exception as e:
    print(repr(e))