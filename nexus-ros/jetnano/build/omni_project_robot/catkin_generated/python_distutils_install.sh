#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/src/omni_project_robot"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/install/lib/python2.7/dist-packages:/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/build" \
    "/usr/bin/python2" \
    "/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/src/omni_project_robot/setup.py" \
     \
    build --build-base "/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/build/omni_project_robot" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/install" --install-scripts="/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/install/bin"
