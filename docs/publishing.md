# Publishing to mul-cps/teaching-exciting-science-2026

The workshop package was built locally. **The ChatGPT GitHub connector returned HTTP 403 (`Resource not accessible by integration`) when writing to the empty `mul-cps` repository.** No remote commit has been made.

## Git push on a workstation with normal GitHub write access

1. Download `teaching-exciting-science-2026.zip` from the ChatGPT answer.
2. Open a terminal and run:

```bash
git clone https://github.com/mul-cps/teaching-exciting-science-2026.git
cd teaching-exciting-science-2026
unzip /path/to/teaching-exciting-science-2026.zip
git add .
git commit -m "Add CPS Candy Factory workshop"
git branch -M main
git push -u origin main
```

The ZIP contains files at the repository root, not inside a nested folder. The GitHub Actions workflow validates the simulator and creates DOCX/PPTX artifacts on each push. The ZIP also contains built DOCX/PPTX files directly.

## Or authorize GitHub connector

An organization administrator needs to grant the ChatGPT GitHub App installation access to `mul-cps` with repository contents write permission. Only then can connector-based publishing be retried. Do not interpret read access as proof of write permission.
