# Tilted
Our attempt at making an AR drawing Google Cardboard project for EE180D. 

Things that we want to work on:
1. Get Madgwick working. (it's clearly not working right now.)
	 If not, we can try doing stuff with our current pose to get the position,
	 maybe. 

2. Add error correction on our IMU.
	 Zero-state pose update.
	 Snapping to previous stop point.
	 Determining (by inference) whether things have stopped or not. 

3. For implementing POST to do server stuff:
	 https://curl.haxx.se/libcurl/c/http-post.html

	 Add this once our Firebase is set up. 