## Externals
- viral (https://resy-gitlab.inf.uni-bayreuth.de/viral/viral.git)
- libfranka (https://resy-gitlab.inf.uni-bayreuth.de/libfranka/libfranka.git)
- eigen (https://gitlab.com/libeigen/eigen.git)
Must be placed next to the checked out repository.

## Usage
- Include
    - /source
- Lib-paths
    - /build/$Platform_$Configuration
- Link
    - franka_control.lib
    - franka_proxy_client.lib