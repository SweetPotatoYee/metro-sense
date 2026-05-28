#!/bin/sh

SCRIPT_DIR=$( cd -- "$( dirname -- "$0" )" >/dev/null 2>&1 && pwd )

exec java \
    -Xmx64m -Xms64m \
    -classpath "$SCRIPT_DIR/gradle/wrapper/gradle-wrapper.jar" \
    org.gradle.wrapper.GradleWrapperMain \
    "$@"
