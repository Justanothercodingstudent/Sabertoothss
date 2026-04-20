# REFINERY RoboRIO MCP Server

This folder contains a deployable MCP client configuration for connecting AI tools to a live RoboRIO.

## File

- `refinery-roborio-mcp.json`

## Usage

1. Copy the JSON into your MCP client config (or point your client at this file directly).
2. Update the `ROBORIO_HOST` to your team IP (example in file uses `10.36.59.2`).
3. Set `ROBORIO_PASSWORD` if your image requires one.
4. Start the MCP server from your MCP-capable AI client.

## Notes

- This file is in `src/main/deploy` so it can be bundled with robot deploy assets.
- The server package name can be swapped if your REFINERY build uses a different npm artifact.
