export TSLIB_CONSOLEDEVICE=none
export TSLIB_FBDEVICE=/dev/fb0

export TSLIB_TSDEVICE=/dev/input/event1
export TSLIB_CONFFILE=/usr/local/tslib/etc/ts.conf
export TSLIB_PLUGINDIR=/usr/local/tslib/lib/ts
export TSLIB_CALIBFILE=/etc/pointercal
export LD_LIBRARY_PATH=/lib:/usr/lib:/usr/local/tslib/lib:/opt/qt-4.8.5/lib
export QT_QWS_FONTDIR=/opt/qt-4.8.5/lib/fonts
export QWS_USB_KEYBOARD=/dev/input/event2
export PATH=/bin:/sbin:/usr/bin/:/usr/sbin:/usr/local/tslib/bin

if grep "hdmi" /proc/cmdline > /dev/null ; then
	export QWS_MOUSE_PROTO="Tslib:/dev/input/event1 MouseMan:/dev/input/mice"
else
	export QWS_MOUSE_PROTO="Tslib:/dev/input/event1"
fi

export QWS_DISPLAY=:1

