<?php

header('Access-Control-Allow-Origin: *');

$servername = "localhost";
$username = "root";
$password = "";
$database = "blindstick";
$tableName = "buttonstatus";

// Establish database connection
$conn = new mysqli($servername, $username, $password, $database);

if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
}

$sql_create_table = "CREATE TABLE IF NOT EXISTS $tableName (
    id INT AUTO_INCREMENT PRIMARY KEY,
    buttonstatus VARCHAR(255),
    return_text VARCHAR(255)
)";

// Create the table if it doesn't exist
$conn->query($sql_create_table);

// Handle POST requests
if ($_SERVER["REQUEST_METHOD"] == "POST") {
    try {
        // Decode JSON data from the request body
        $data = json_decode(file_get_contents('php://input'), true);

        // Check if the required fields are present
        if (isset($data['buttonstatus']) && isset($data['return_text'])) {
            $originalText = $data['buttonstatus'] ?? '';
            $correctedText = $data['return_text'] ?? '';

            // Check if a record with id=1 exists
            $sql_check_id = "SELECT id FROM $tableName WHERE id = 1";
            $result = $conn->query($sql_check_id);
            if ($result->num_rows > 0) {
                // Update the existing record
                $sql_update = "UPDATE $tableName SET buttonstatus='$originalText', return_text='$correctedText' WHERE id=1";
                if ($conn->query($sql_update) === TRUE) {
                    echo json_encode(['id' => 1, 'message' => 'Data updated successfully']);
                } else {
                    throw new Exception('Failed to update data');
                }
            } else {
                // Insert a new record
                $sql_insert = "INSERT INTO $tableName (buttonstatus, return_text) VALUES ('$originalText', '$correctedText')";
                if ($conn->query($sql_insert) === TRUE) {
                    $insertedId = $conn->insert_id;
                    echo json_encode(['id' => $insertedId, 'message' => 'New data created successfully']);
                } else {
                    throw new Exception('Failed to insert data');
                }
            }
        } else {
            throw new Exception('Missing required fields');
        }
    } catch (Exception $e) {
        // Handle any exceptions
        http_response_code(500);
        echo json_encode(['error' => $e->getMessage()]);
    }
} else if ($_SERVER["REQUEST_METHOD"] == "GET") {
    // Handle GET requests
    $sql_fetch = "SELECT return_text FROM $tableName WHERE id = 1";
    $result = $conn->query($sql_fetch);
    if ($result->num_rows > 0) {
        $row = $result->fetch_assoc();
        echo json_encode(['return_text' => $row['return_text']]);
    } else {
        http_response_code(404);
        echo json_encode(['error' => 'No data found']);
    }
} else {
    // Method Not Allowed for non-POST requests
    http_response_code(405);
    echo json_encode(['error' => 'Invalid request method']);
}

// Close the database connection
$conn->close();

?>
