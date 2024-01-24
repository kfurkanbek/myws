# myws
my workspace

### to forward gui apps to main desktop:
### first in terminal: "xhost +local:"
```
{
  "name": "my ros2 humble desktop",
  "dockerFile": "Dockerfile",
  "runArgs": [
    "--privileged",
    "--network=host"
  ],
  "workspaceMount": "source=${localWorkspaceFolder},target=/${localWorkspaceFolderBasename},type=bind",
  "workspaceFolder": "/${localWorkspaceFolderBasename}",
  "mounts": [
    "source=${localEnv:HOME}${localEnv:USERPROFILE}/.bash_history,target=/home/vscode/.bash_history,type=bind"
    //"source=/home/dante/Desktop,target=/Desktop,type=bind"
  ]
}
```

### to forward gui apps to VNC "localhost:6080":
```
{
  "name": "my ros2 humble desktop",
  "dockerFile": "Dockerfile",
  "runArgs": [
    "--privileged"
    // "--network=host"
  ],
  "workspaceMount": "source=${localWorkspaceFolder},target=/${localWorkspaceFolderBasename},type=bind",
  "workspaceFolder": "/${localWorkspaceFolderBasename}",
  "mounts": [
    "source=${localEnv:HOME}${localEnv:USERPROFILE}/.bash_history,target=/home/vscode/.bash_history,type=bind"
    //"source=/home/dante/Desktop,target=/Desktop,type=bind"
  ],
  "features": {
    "ghcr.io/devcontainers/features/desktop-lite:1": {}
  },
  "portsAttributes": {
    "6080": {
      "label": "Desktop (Web)"
    },
    "5901": {
      "label": "Desktop (VNC)"
    }
  }
}
```
