<?php
$imageDir = "C:/xampp/htdocs/doorbell/ReplaceImage/";

$imageUrls = [];

// Get all files from the uploads directory
$files = scandir($imageDir);

foreach ($files as $file) {
    // Skip . and .. directories
    if ($file == '.' || $file == '..') {
        continue;
    }

    // Construct the URL for each image file
    $imageUrl = 'http://' . $_SERVER['HTTP_HOST'] . '/doorbell/ReplaceImage/' . $file;

    // Add the URL to the array of image URLs
    $imageUrls[] = $imageUrl;
}

// Output the array of image URLs as JSON
echo json_encode($imageUrls);
?>
