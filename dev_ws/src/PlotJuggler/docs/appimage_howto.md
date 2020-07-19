# Build the AppImage with catkin_make

In the root folder of ws_plotjuggler:

    wget https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage
    wget https://github.com/linuxdeploy/linuxdeploy-plugin-qt/releases/download/continuous/linuxdeploy-plugin-qt-x86_64.AppImage

    chmod +x linuxdeploy*.AppImage

    rm -rf build devel install AppDir
    catkin_make -DCMAKE_BUILD_TYPE=Release  -j$(nproc) install  
    
    cd src/PlotJuggler;export VERSION=$(git describe --abbrev=0 --tags);cd -;echo $VERSION
    
    mkdir -p AppDir/usr/bin/
    cp install/lib/plotjuggler/* AppDir/usr/bin
    
    ./linuxdeploy-x86_64.AppImage --appdir=AppDir -d src/PlotJuggler/PlotJuggler.desktop -i src/PlotJuggler/plotjuggler-icon.png  --plugin qt --output appimage
     
    


