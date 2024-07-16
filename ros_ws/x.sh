#!/bin/bash

# 기본값 설정
DEFAULT_PKG="auto_runner"
DEFAULT_BUILD="one"
DEFAULT_LAUNCH="launch2"
DEFAULT_TYPE="launch"

# 사용법 함수
usage() {
    echo "Usage: $0 [-p package] [-b build] [-l launch] [-t type]"
    echo "  -p: Package name (default: ${DEFAULT_PKG})"
    echo "  -b: Build option (default: ${DEFAULT_BUILD})"
    echo "  -l: Launch option (default: ${DEFAULT_LAUNCH})"
    exit 1
}
do_import() {
    if [ -f install/setup.bash ]; then
        source install/setup.bash
    else
        echo "Error: install/setup.bash not found"
	exit 1
    fi
}

# 인자 파싱
unset type
while (( $# )); do
    case "$1" in
        -p) shift; pkg="$1" ;;
        -b) shift; build=$1; type=build ;;
        -l) shift; launch="$1"; type=launch ;;
        -s) do_import; echo "Load source"; exit 0;;
        *) usage ;;
    esac
    shift
done

if [[ -z "$type" ]]; then
  echo "sourcing"
  exit
fi
# 변수 설정 (기본값 사용)
PKG=${pkg:-$DEFAULT_PKG}
BLD=${build:-$DEFAULT_BUILD}
LAUNCH=${launch:-$DEFAULT_LAUNCH}
TYP=${type:-$DEFAULT_TYPE}

# 결과 출력
echo "Package: $PKG"
echo "Build: $BLD"
echo "Launch: $LAUNCH"
echo "Type: $TYP"

# 빌드 함수
do_build() {
    if [ "$BLD" = "all" ]; then
        colcon build
        echo "Building all packages"
    else
        colcon build --packages-select "$PKG"
        echo "Building $PKG package"
    fi
}

# 실행 함수
do_launch() {
    #echo "Launching: $PKG ${PKG}.${LAUNCH}.py"
    ros2 launch "$PKG" "${PKG}.${LAUNCH}.py"
}

# sourcing
do_import

# 메인 로직
if [ "$TYP" = "build" ]; then
    do_build
    source install/setup.bash
elif [ "$TYP" = "launch" ]; then
    do_launch
else
    echo "Error: Invalid type. Must be 'build' or 'launch'."
    usage
fi
