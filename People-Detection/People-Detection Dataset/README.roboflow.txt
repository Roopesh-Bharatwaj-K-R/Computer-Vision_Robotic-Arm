
People Detection - v1 People Detection 1
==============================

This dataset was exported via roboflow.ai on April 4, 2022 at 1:50 PM GMT

It includes 625 images.
Letters are annotated in YOLO v5 PyTorch format.

The following pre-processing was applied to each image:
* Auto-orientation of pixel data (with EXIF-orientation stripping)
* Resize to 416x416 (Stretch)

The following augmentation was applied to create 3 versions of each source image:
* 50% probability of horizontal flip
* Randomly crop between 0 and 20 percent of the image
* Random rotation of between -15 and +15 degrees
* Random brigthness adjustment of between -25 and +25 percent

The following transformations were applied to the bounding boxes of each image:
* 50% probability of horizontal flip
* Random shear of between -15° to +15° horizontally and -15° to +15° vertically


