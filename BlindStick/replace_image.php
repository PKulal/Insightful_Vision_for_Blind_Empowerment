<?php
// Check if image file is an actual image or fake image
if(isset($_FILES['image'])) {
    $target_dir = "C:/xampp/htdocs/doorbell/ReplaceImage/"; // Adjust the path as needed
    $uploadOk = 1;
    $imageFileType = strtolower(pathinfo($_FILES["image"]["name"], PATHINFO_EXTENSION));
    
    // Check file size
    if ($_FILES["image"]["size"] > 500000) {
        echo "Sorry, your file is too large.";
        $uploadOk = 0;
    }
    // Allow certain file formats
    if($imageFileType != "jpg" && $imageFileType != "png" && $imageFileType != "jpeg"
    && $imageFileType != "gif" ) {
        echo "Sorry, only JPG, JPEG, PNG & GIF files are allowed.";
        $uploadOk = 0;
    }
    // Check if $uploadOk is set to 0 by an error
    if ($uploadOk == 0) {
        echo "Sorry, your file was not uploaded.";
    } else {
        // Remove previous image if exists
        $existingFiles = glob($target_dir . "*");
        foreach ($existingFiles as $file) {
            if (is_file($file)) {
                unlink($file);
            }
        }

        // Upload the new image
        if (move_uploaded_file($_FILES["image"]["tmp_name"], $target_dir . $_FILES["image"]["name"])) {
            echo "The file ". htmlspecialchars( basename( $_FILES["image"]["name"])). " has been uploaded.";
        } else {
            echo "Sorry, there was an error uploading your file.";
        }
    }
} else {
    echo "No image uploaded.";
}
?>
