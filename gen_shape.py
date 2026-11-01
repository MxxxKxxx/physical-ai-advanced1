import json
import os
import sys

from google import genai
from google.genai import types

MODEL = os.environ.get('GEMINI_MODEL', 'gemini-3.6-flash')
OUT_DIR = os.path.expanduser('~/draw_bot')

PROMPT = """あなたは、床に一筆書きで絵を描く小型ロボットの作画担当です。
お題：「{theme}」

ルール：
- 一筆書きの折れ線（点の列）で表現する。途中でペンを持ち上げることはできない。
- 座標の単位はメートル。x, y とも -1.0〜1.0 の範囲に収める。
- 点の数は8〜40個。隣り合う点の間隔は0.05m以上あける。
- 細かすぎる描写は避け、特徴が伝わる簡潔な形にする。
- 出力はJSONのみ：
{{"title": "作品名", "idea": "どう表現したか（日本語で1文）", "points": [[x, y], ...]}}
"""


def clamp(v):
    return max(-1.0, min(1.0, float(v)))


def save_preview(data, path):
    """生成した図形をPNGで保存（レポート・動画用）"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    xs = [p[0] for p in data['points']]
    ys = [p[1] for p in data['points']]
    plt.figure(figsize=(4, 4))
    plt.plot(xs, ys, '-o', color='red', markersize=3)
    plt.plot(xs[0], ys[0], 'go', markersize=8)  # 始点は緑
    plt.gca().set_aspect('equal')
    plt.xlim(-1.1, 1.1)
    plt.ylim(-1.1, 1.1)
    plt.grid(alpha=0.3)
    plt.savefig(path, dpi=120)


def main():
    theme = ' '.join(sys.argv[1:]) or '星'
    client = genai.Client()  # 環境変数 GEMINI_API_KEY を使う
    res = client.models.generate_content(
        model=MODEL,
        contents=PROMPT.format(theme=theme),
        config=types.GenerateContentConfig(
            response_mime_type='application/json', temperature=0.7),
    )
    data = json.loads(res.text)
    data['points'] = [[clamp(x), clamp(y)] for x, y in data['points']]
    if len(data['points']) < 2:
        sys.exit('点が少なすぎます。もう一度実行してください。')
    data['theme'] = theme

    with open(os.path.join(OUT_DIR, 'shape.json'), 'w') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    save_preview(data, os.path.join(OUT_DIR, 'shape_preview.png'))

    print(f"お題：{theme}")
    print(f"作品名：{data.get('title')}")
    print(f"AIの表現意図：{data.get('idea')}")
    print(f"点の数：{len(data['points'])} → ~/draw_bot/shape.json に保存")


if __name__ == '__main__':
    main()
