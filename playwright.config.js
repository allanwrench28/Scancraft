import { defineConfig } from '@playwright/test';
import { homedir } from 'node:os';
import { join } from 'node:path';

const python = process.env.SCANCRAFT_PYTHON || join(homedir(), 'miniconda3', process.platform === 'win32' ? 'python.exe' : 'bin/python');

export default defineConfig({
  testDir: './gui/tests',
  outputDir: './target/gui-validation/browser',
  timeout: 120000,
  workers: 1,
  reporter: 'list',
  projects: [
    { name: 'browser', testIgnore: '**/packaged.spec.js' },
    { name: 'packaged', testMatch: '**/packaged.spec.js' },
  ],
  use: {
    baseURL: 'http://127.0.0.1:4318',
    browserName: 'chromium',
    channel: process.platform === 'win32' ? 'msedge' : 'chromium',
    viewport: { width: 1360, height: 900 },
    screenshot: 'only-on-failure',
    trace: 'retain-on-failure',
  },
  webServer: {
    command: `"${python}" scancraft_gui.py --no-browser --port 4318`,
    url: 'http://127.0.0.1:4318',
    timeout: 60000,
    reuseExistingServer: !process.env.CI,
  },
});