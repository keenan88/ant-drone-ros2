# Use an official Node.js image as a base
FROM node:16

# Set the working directory
WORKDIR /usr/src/app

# Install git to clone the repository
RUN apt-get update && apt-get install -y git && rm -rf /var/lib/apt/lists/*

# Clone the specific version (1.4.1) of the roslibjs repository
RUN git clone --branch 1.4.1 https://github.com/RobotWebTools/roslibjs.git .

# # Install dependencies
# RUN npm install

# # Build the project
# RUN npm run build

# # Install a lightweight web server to serve the files
# RUN npm install -g http-server

# # Expose the port for the web server
# EXPOSE 8080

# # Start the server to serve the built files
# CMD ["http-server", "build", "-p", "8080"]
