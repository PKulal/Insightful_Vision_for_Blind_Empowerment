<?php
if(isset($_FILES['image'])) {
    $target_dir = "C:/xampp/htdocs/BlindStick/ReplaceImage/";
    $uploadOk = 1;
    $imageFileType = strtolower(pathinfo($_FILES["image"]["name"], PATHINFO_EXTENSION));

    if ($_FILES["image"]["size"] > 500000) {
        echo "Sorry, your file is too large.";
        $uploadOk = 0;
    }

    if($imageFileType != "jpg" && $imageFileType != "png" && $imageFileType != "jpeg"
    && $imageFileType != "gif" ) {
        echo "Sorry, only JPG, JPEG, PNG & GIF files are allowed.";
        $uploadOk = 0;
    }

    if ($uploadOk == 0) {
        echo "Sorry, your file was not uploaded.";
    } else {
        $existingFiles = glob($target_dir . "*");
        foreach ($existingFiles as $file) {
            if (is_file($file)) {
                unlink($file);
            }
        }

        if (move_uploaded_file($_FILES["image"]["tmp_name"], $target_dir . $_FILES["image"]["name"])) {
            echo "The file ". htmlspecialchars( basename( $_FILES["image"]["name"])). " has been uploaded.";

            $pythonScript = "python C:/xampp/htdocs/safety_car/face_recognition_script.py " . $target_dir . $_FILES["image"]["name"];
            $result = exec($pythonScript);
            if ($result == "Face found") {
                echo "Face recognized!";
            } else {
                echo "Face not recognized.";
            }
        } else {
            echo "Sorry, there was an error uploading your file.";
        }
    }
} else {
    echo "No image uploaded.";
}
?>
