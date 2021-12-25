from PIL import Image, ImageDraw, ImageFont


def get_wrapped_text(text: str, font: ImageFont.ImageFont,
                     line_length: int):
        lines = ['']
        for word in text.split():
            line = f'{lines[-1]} {word}'.strip()
            if font.getsize(line)[0] <= line_length:
                lines[-1] = line
            else:
                lines.append(word)
        return '\n'.join(lines)


font = ImageFont.load_default()
#text = 'An example line of text that will need to be wrapped.'
#wrapped_text = get_wrapped_text(text, font, line_length=70)
#print(wrapped_text)

width = 128
height = 32
image = Image.new("1", (width, height))

draw = ImageDraw.Draw(image)

text = "Salut Antoine, comment vas-tu ?"

print(get_wrapped_text(text, font, width))