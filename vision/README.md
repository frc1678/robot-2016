** documentation for vision code **

* vision_main *
open camera, start thread with `DataSender`, run `OnjectDetector` and send it to `DataSender`.

* data_sender *
Run `startSending` to continuously send data over the network to the roborio. Run `updateData` to set the data from the vision processing.

* object_detector *
Run `update` to do the processing on a image.

* shape_detector *
run `setData` to find the target in a black-and-white image. `getPoints` and `getScore` gets information about it. `getAllContours` gets all the contours from an image.

* in_range_instructions *
Can apply thresh to an image, read it from a file, write it to a file, or be given specific values.
