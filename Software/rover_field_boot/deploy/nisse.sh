# Nisse environment — sourced by every interactive shell.
# Placed at /etc/profile.d/nisse.sh by install.sh.

export NISSE_ROOT=/opt/nisse

if [ -f "$NISSE_ROOT/venv/bin/activate" ]; then
    # Activate without turning on the bash prompt marker — keeps field
    # consoles clean.
    VIRTUAL_ENV_DISABLE_PROMPT=1 . "$NISSE_ROOT/venv/bin/activate"
fi

case ":$PATH:" in
    *":$NISSE_ROOT/bin:"*) ;;
    *) export PATH="$NISSE_ROOT/bin:$PATH" ;;
esac

if [ -f "$NISSE_ROOT/etc/welcome.txt" ]; then
    cat "$NISSE_ROOT/etc/welcome.txt"
fi
