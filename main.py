"""
This script prints concise git push instructions for a local repo.
"""

def print_git_push_steps(remote_url_placeholder="<REPO_URL>", branch="main"):
    steps = [
        "1) (Optional) Configure user if not set:",
        "   git config --global user.name \"Your Name\"",
        "   git config --global user.email \"you@example.com\"",
        "",
        "2) Check repo status:",
        "   git status",
        "",
        "3) Stage all changes:",
        "   git add .",
        "",
        "4) Commit:",
        "   git commit -m \"Your commit message\"",
        "",
        f"5) Ensure branch name (optional):",
        f"   git branch -M {branch}",
        "",
        "6) Add remote (only if not added):",
        f"   git remote add origin {remote_url_placeholder}",
        "",
        f"7) Push and set upstream:",
        f"   git push -u origin {branch}",
        "",
        "If remote already exists, run:",
        "   git remote -v",
        "   git push origin <branch>",
        "",
        "Authentication tips:",
        "- For HTTPS use a personal access token if prompted for a password.",
        "- For SSH, add your SSH public key to your GitHub account and use the SSH repo URL."
    ]
    print("\\n".join(steps))

if __name__ == "__main__":
    # Replace <REPO_URL> with your repo HTTPS or SSH URL, and branch name if different.
    print_git_push_steps()