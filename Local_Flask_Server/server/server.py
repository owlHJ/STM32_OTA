from flask import Flask, Response, request, send_from_directory, render_template, redirect, url_for
import os

app = Flask(__name__)

# Set the folder to store uploaded files
UPLOAD_FOLDER = os.path.join(os.getcwd(), 'uploads')
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
print("http://localhost:5000/")

# Create the uploads folder if it doesn't exist
if not os.path.exists(UPLOAD_FOLDER):
    os.makedirs(UPLOAD_FOLDER)

# Main page
@app.route('/')
def index():
    # Check if firmware.hex exists in the uploads folder
    firmware_file = None
    for file in os.listdir(app.config['UPLOAD_FOLDER']):
        if file == "firmware.hex":
            firmware_file = file
            break
    return render_template('index.html', filename=firmware_file)

# Handle file upload
@app.route('/upload', methods=['POST'])
def upload_file():
    if 'file' not in request.files:
        return "No file part"
    
    file = request.files['file']
    if file.filename == '':
        return "No selected file"
    
    if file:
        # Save the uploaded file as firmware.hex, overwriting if necessary
        file.save(os.path.join(app.config['UPLOAD_FOLDER'], 'firmware.hex'))
        return redirect(url_for('index'))

# Stream the firmware file line-by-line for download
@app.route('/download/firmware', methods=['GET'])
def download_firmware():
    def generate():
        firmware_path = os.path.join(app.config['UPLOAD_FOLDER'], 'firmware.hex')
        if os.path.exists(firmware_path):
            with open(firmware_path, 'r') as f:
                for line in f:
                    yield line  # Send each line as a chunk
        else:
            yield "File not found\n"

    return Response(generate(), mimetype='text/plain')  # Send the response as a text stream

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
