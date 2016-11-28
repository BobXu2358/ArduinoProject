#Processing
- Keep pressing keyboard to accelerate<br>
- Keyboard keys "w", "s", 'a' and "d" corresponds to forward, backward, left and right<br>
- Release key to stop<br>

#Special note for usage
- Do not open serial monitor otherwsie the processing application will not run because the serial is occupied by monitor<br>
- Close the processing application window before you upload Arduino code to board otherwise the uploading will fail<br>

#Note for issues so far
- Problem with sample data I talked in email. Until test with device I cannot find furthur defects<br>
- The Arduino serial frequency and Processing serial frequency are asynchronized! This is OK with keyboard controlling. However this dmages the string data written by Arduino code and read by Processing so we must deal with this
