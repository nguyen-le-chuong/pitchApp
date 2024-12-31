# Pitch Estimation

---

## Prerequisites

1. **Java Development Kit (JDK):**  
   Install the latest version of the JDK from [Adoptium](https://adoptium.net/) or [Oracle](https://www.oracle.com/java/technologies/javase-downloads.html).

2. **Command-Line Tools:**  
   Download the Android Command-Line Tools from the [Android Studio download page](https://developer.android.com/studio#command-tools).

---

## Install Android Studio
Install NDK and Cmake on Android Studio Settings
## Install Command-Line Tools (Optional)

### 1.1 Download and Extract Tools
Extract the command-line tools into a directory (e.g., `~/android-sdk`).

### 1.2 Install SDK and NDK Components
Run the following commands:
```bash
cd ~/android-sdk/cmdline-tools/bin

# Update SDK Manager
./sdkmanager --update

# Install essential components
./sdkmanager "platform-tools" "build-tools;34.0.0" "platforms;android-33" "ndk;25.2.9519653" "cmake;3.22.1"

```
## Set Environment Variables
```bash
export ANDROID_HOME=~/android-sdk
export PATH=$ANDROID_HOME/cmdline-tools/bin:$ANDROID_HOME/platform-tools:$PATH
export PATH=$ANDROID_HOME/ndk/25.2.9519653:$PATH
```

custom your `local.properties`
## Build and Run the Project
```
./gradlew build
```
## Files structure
Pitch Estimation: `app/src/main/cpp/kalmanfilter.cpp`
Road Slope Estimation: `app/src/main/cpp/filterroad.cpp`
