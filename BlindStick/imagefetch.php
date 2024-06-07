<?php
// Set the appropriate content type for the response
header('Content-Type: application/json');

// Path to your image file
$imagePath = 'C:/xampp/htdocs/doorbell/ReplaceImage/replaceImage.jpg'; // Update with your image file path

// Read the image data
$imageData = file_get_contents($imagePath);

if ($imageData !== false) {
    // Encode the image data to base64
    $base64Image = base64_encode($imageData);
    
    // Return the base64 encoded image data as JSON
    echo json_encode(array('image' => $base64Image));
} else {
    // If the file doesn't exist or couldn't be read, return an error message
    echo json_encode(array('error' => 'Image not found'));
}
?>
