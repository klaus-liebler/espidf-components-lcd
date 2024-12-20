import { Font } from 'opentype.js';

export const UNICODE_PRIVATE_USE_AREA = 57344; //57344
export const FONT_UNITS_PER_PIXEL = 128;

export class Range {
	constructor(public readonly startIncl: number, public readonly endExcl: number) { }
}
export class GlyphProcessingInfo {
	constructor(public readonly codepointSource: number, public readonly codepointDest: number, public readonly glyphIndexSource: number, public readonly glyphIndexDest: number, public readonly glyphName: string, public readonly font: Font) { }
}

export class CharacterMap0Tiny {

	constructor(public readonly firstCodepoint: number, public readonly firstGlyph: number, public len = 0) { }
	public toCppConstructorString(): string {
		return `CharacterMap0Tiny(${this.firstCodepoint}, ${this.firstGlyph}, ${this.len})`;
	}
}

export enum BitmapFormat{
    UNDEFINED,
    ONE_BPP_EIGHT_IN_A_COLUMN,//best for SSD1306 etc
    FOUR_BPP_ROW_BY_ROW,//best for ST7789 etc
    ONE_BPP_ROW_BY_ROW,
}

export class GlyphDesc {
	constructor(public readonly codepointDest:number, public readonly name:string, public adv_w = 0, public box_w = 0, public box_h = 16, public ofs_x = 0, public ofs_y = 0, public kerningClassLeft=0, public kerningClassRight=0, public readonly bitmapFormat:BitmapFormat, public readonly bitmap:Uint8Array) { }

	public toCppConstructorString(bitmap_index: number, glyphIndex:number): string {
        //        GlyphDesc(uint32_t bitmap_index, uint16_t adv_w, uint8_t box_w, uint8_t box_h, uint8_t ofs_x, uint8_t ofs_x)
		return `\tGlyphDesc(${bitmap_index}, ${this.adv_w}, ${this.box_w}, ${this.box_h}, ${this.ofs_x}, ${this.ofs_y}, ${this.kerningClassLeft}, ${this.kerningClassRight}, lcd_common::BitmapFormat::${BitmapFormat[this.bitmapFormat]}),// ${this.name} glyphindex=${glyphIndex} codepoint=${this.codepointDest}\n`;
	}
}

export function concat(a: Array<Uint8Array>): Uint8Array {
	var d = new Uint8Array(a.reduce<number>((total, value)=>total+value.length, 0));
    var pos=0;
    for(var i=0;i<a.length;i++){
        d.set(a[i], pos)
        pos+=a[i].length;
    }
	return d;
}

export function ToUint8Array(...a: Array<Array<number>>): Uint8Array {
	var d = new Uint8Array(a.reduce((pv, cv, ci, a)=>pv+cv.length, 0));
    var start=0;
    a.forEach((v=>{
        d.set(v, start);
        start+=v.length;
    }))
	return d;
}

export function formatedTimestamp() {
    const d = new Date()
    const date = d.toISOString().split('T')[0];
    const time = d.toTimeString().split(' ')[0];
    return `${date} ${time}`
}

export class GlyphProviderResult{
    //public fontUnitsPerPixel:number; --> Nein, Wir normalisieren und gehen immer von 128 Einheiten pro Pixel aus
    public lineHeight:number;
    public glyphsDesc:Array<GlyphDesc>;
}

export class GlyphProviderWithKerningResult{
    public lineHeight:number;
    public glyphsDesc:Array<GlyphDesc>;
    //public fontUnitsPerPixel:number; --> Nein, Wir normalisieren und gehen immer von 128 Einheiten pro Pixel aus
    public leftKerningClassCnt:number;
    public rightKerningClassCnt:number;
    public leftrightKerningClass2kerningValue:Array<number>;
}



export interface IGlyphProviderWithKerning{
    Provide():Promise<GlyphProviderWithKerningResult>;
}

export interface IGlyphProvider{
    Provide():Promise<GlyphProviderResult>;
}

