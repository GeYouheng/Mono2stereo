# Mono2stereo
This package converts the EMR stereo camera's original image to 2 seperate images. 

## Input and output
Subscribe:
/usb_cam/image_raw

Publish:
/mono2stereo/left_original
/mono2stereo/right_original
/mono2stereo/left_360p
/mono2stereo/right_360p
/mono2stereo/left_rect
/mono2stereo/right_rect

## Note
The rectification is done according to pre-calibrated parameters. 
You can modify them in "setting_left_360p.yaml" and "setting_left_360p.yaml".
