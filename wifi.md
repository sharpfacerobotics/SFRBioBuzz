1. Connect to the robot's Wi-Fi.
2. Open the terminal.
3. The paths listed below are absolute paths, but of course, you can always use a relative path if you wish.

   For Windows machines run, `C:/Users/<user name>/AppData/Local/Android/Sdk/platform-tools/adb.exe connect 192.168.43.1:5555`.
   If you have administrative privileges, you can add the filepath of ADB to the `PATH` environment variable. Then you can simply run `adb connect 192.168.43.1:5555` to connect to the robot.

   For Mac machines, run `/Users/<user name>/Library/Android/sdk/platform-tools/adb connect 192.168.43.1:5555` or `adb connect 192.168.43.1:5555`.
   Unless you enjoy typing, it is recommended to use `adb`.

4. Once you are connected, go to Android Studio and click the upload code button.
5. Wait a minute, and your code should not be uploaded.
