from __future__ import annotations

import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import better_outer_wall_processing as plugin


BASE_DIR = Path(__file__).resolve().parent
MANUAL_PATH = BASE_DIR / "使用说明书.md"


def load_config() -> dict:
    return plugin.load_config()


def save_config(data: dict) -> None:
    plugin.save_config(data)


class App:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("更好的外墙处理设置")
        self.root.geometry("980x760")
        self.config = load_config()
        self.vars: dict[str, tk.Variable] = {}

        container = ttk.Frame(root, padding=12)
        container.pack(fill="both", expand=True)

        left = ttk.Frame(container)
        left.pack(side="left", fill="y")

        right = ttk.Frame(container)
        right.pack(side="right", fill="both", expand=True, padx=(12, 0))

        form = ttk.Frame(left)
        form.pack(fill="y")

        for row, field in enumerate(plugin.CONFIG_FIELDS):
            key = field["key"]
            kind = field["kind"]
            ttk.Label(form, text=field["label"]).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=4)
            if kind == "bool":
                var = tk.BooleanVar(value=bool(self.config.get(key)))
                widget = ttk.Checkbutton(form, variable=var)
            elif kind == "enum":
                default_value = plugin.DEFAULT_CONFIG.get(key, "")
                var = tk.StringVar(value=str(self.config.get(key, default_value)))
                widget = ttk.Combobox(form, textvariable=var, values=field["options"], state="readonly", width=14)
            else:
                var = tk.StringVar(value=str(self.config.get(key, "")))
                widget = ttk.Entry(form, textvariable=var, width=16)
            widget.grid(row=row, column=1, sticky="ew", pady=4)
            self.vars[key] = var

        button_bar = ttk.Frame(left)
        button_bar.pack(fill="x", pady=(12, 0))
        ttk.Button(button_bar, text="保存", command=self.on_save).pack(side="left")
        ttk.Button(button_bar, text="重新加载", command=self.on_reload).pack(side="left", padx=8)
        ttk.Button(button_bar, text="关闭", command=self.root.destroy).pack(side="left")

        ttk.Label(right, text="中文说明", font=("Microsoft YaHei UI", 11, "bold")).pack(anchor="w", pady=(0, 8))
        self.text = tk.Text(right, wrap="word")
        self.text.pack(fill="both", expand=True)
        self.text.insert("1.0", MANUAL_PATH.read_text(encoding="utf-8"))
        self.text.config(state="disabled")

    def on_reload(self) -> None:
        self.config = load_config()
        for field in plugin.CONFIG_FIELDS:
            key = field["key"]
            kind = field["kind"]
            value = self.config.get(key)
            if kind == "bool":
                self.vars[key].set(bool(value))
            else:
                self.vars[key].set(str(value))

    def on_save(self) -> None:
        data = dict(self.config)
        try:
            for field in plugin.CONFIG_FIELDS:
                key = field["key"]
                data[key] = plugin.coerce_config_value(field["kind"], self.vars[key].get())
            save_config(data)
            self.config = data
            messagebox.showinfo("更好的外墙处理", "配置已保存。")
        except Exception as exc:
            messagebox.showerror("保存失败", str(exc))


def main() -> None:
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
