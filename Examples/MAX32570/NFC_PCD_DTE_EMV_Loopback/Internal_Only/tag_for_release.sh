#!/bin/sh

NEW_TAG="v4.2.0_RC1"
TAG_MESSAGE="Release $NEW_TAG"
PATH_TO_PBM="../../../../Libraries/NFC/lib_nfc_pcd_pbm"
PATH_TO_RF_DRIVER="../../../../Libraries/NFC/lib_nfc_pcd_rf_driver_MAX32570"
PATH_TO_MSDK="../../../../"
PATH_TO_INSTALLER="/c/prog/me13/msdk_2_5_2020_workspace/me13_nfc_installer/nfc"

tag_push_check () {
    echo
    read -p "Push Tag for $1 [y/n]?" -n 1 -r
    echo    # (optional) move to a new line
    if [[ $REPLY =~ ^[Yy]$ ]]
    then
        return
    else
        echo "Quiting tag process"
        exit
    fi
}

echo "Tagging DTE and Stack"
git tag -a $NEW_TAG -m "$TAG_MESSAGE"

echo "Tagging PBM"
pushd $PATH_TO_PBM
git tag -a $NEW_TAG -m "$TAG_MESSAGE"
popd

echo "Tagging RF Driver"
pushd $PATH_TO_RF_DRIVER
git tag -a $NEW_TAG -m "$TAG_MESSAGE"
popd

echo "Tagging MSDK"
pushd $PATH_TO_MSDK
git tag -a $NEW_TAG -m "$TAG_MESSAGE"
popd

echo "Tagging Installer"
pushd $PATH_TO_INSTALLER
git tag -a $NEW_TAG -m "$TAG_MESSAGE"
popd

echo "Pushing these tags"

git log -n 3
tag_push_check "DTE and Stack"
git push
git push origin $NEW_TAG

pushd $PATH_TO_PBM
git log -n 3
tag_push_check "PBM"
git push
git push origin $NEW_TAG
popd

pushd $PATH_TO_RF_DRIVER
git log -n 3
tag_push_check "RF_DRIVER"
git push
git push origin $NEW_TAG
popd

pushd $PATH_TO_MSDK
git log -n 3
tag_push_check "MSDK"
git push
git push origin $NEW_TAG
popd

pushd $PATH_TO_INSTALLER
git log -n 3
tag_push_check "Installer"
git push
git push origin $NEW_TAG
popd

echo
echo "All Done tagging and pushing"
echo
echo "Make sure Version_History.txt is up to date in nfc_release_packet"
