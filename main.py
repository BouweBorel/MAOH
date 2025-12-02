# Main file

def print_git_model_add_instructions(repo_root=".", target_dir="models"):
    """
    Prints commands to inspect and add a 'models' directory to git.
    Use from Simulation/ directory or from repo root as needed.
    """
    cmds = [
        "# From repo root (recommended):",
        "cd /home/borel-bouwe/Dev/MAOH",
        f"git status",
        f"git add {target_dir}",
        'git commit -m "Add models folder"',
        "git push",
        "",
        "# If you are inside Simulation/ and models is ../models:",
        "git add ../models",
        "git commit -m \"Add models folder\"",
        "git push",
        "",
        "# If the folder is empty, create a placeholder:",
        f"touch {target_dir}/.gitkeep",
        f"git add {target_dir}/.gitkeep",
        "",
        "# To see if files are ignored:",
        "git check-ignore -v ../models/*   # adjust path as needed",
        "",
        "# To force-add an ignored file:",
        "git add -f <path/to/file>",
    ]
    print("\\n".join(cmds))

if __name__ == "__main__":
    print_git_model_add_instructions()