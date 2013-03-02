//
// GLUTsaveBMP
// Bitmap.cpp
//
// The MIT License
//
// Copyright (c) 2009 sonson, sonson@Picture&Software
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////
//
//	Bitmapファイル書き込み
//
///////////////////////////////////////////////////////////////////////////////

#include "Bitmap.h"

///////////////////////////////////////////////////////////////////////////////
//Bitmap構造体初期化
void InitHeaders(BitmapHeader *header,BitmapInfoHeader *info){
	header->distinct1 = 'B';
	header->distinct2 = 'M';
	header->filesize = 0;
	header->reserve1 = 0;
	header->reserve2 = 0;
	header->offset = 54;

	info->header = 40;
	info->width = 0;
	info->height = 0;
	info->plane = 1;
	info->bits = 24;
	info->compression = 0;
	info->comp_image_size = 0;
	info->x_resolution = 0;
	info->y_resolution = 0;
	info->pallet_num = 0;
	info->important_pallet_num = 0;
};
///////////////////////////////////////////////////////////////////////////////
//Bitmapヘッダ書き込み
void WriteHeader(BitmapHeader *header,FILE *fp){
	fwrite(&(header->distinct1), sizeof(char),1,fp);
	fwrite(&(header->distinct2), sizeof(char),1,fp);
	fwrite(&(header->filesize), sizeof(int),1,fp);
	fwrite(&(header->reserve1), sizeof(short),1,fp);
	fwrite(&(header->reserve2), sizeof(short),1,fp);
	fwrite(&(header->offset), sizeof(int),1,fp);
}
///////////////////////////////////////////////////////////////////////////////
//BitmapInfoヘッダ書き込み
void WriteInfoHeader(BitmapInfoHeader *info,FILE *fp){
	fwrite(&(info->header), sizeof(int),1,fp);
	fwrite(&(info->width), sizeof(int),1,fp);
	fwrite(&(info->height), sizeof(int),1,fp);
	fwrite(&(info->plane), sizeof(short),1,fp);
	fwrite(&(info->bits), sizeof(short),1,fp);
	fwrite(&(info->compression), sizeof(int),1,fp);
	fwrite(&(info->comp_image_size), sizeof(int),1,fp);
	fwrite(&(info->x_resolution), sizeof(int),1,fp);
	fwrite(&(info->y_resolution), sizeof(int),1,fp);
	fwrite(&(info->pallet_num), sizeof(int),1,fp);
	fwrite(&(info->important_pallet_num), sizeof(int),1,fp);
}
///////////////////////////////////////////////////////////////////////////////
//ピクセルデータをBitmapファイルへ書き込む
int WriteBitmapFromGL(const char* filename, const int & viewport_x, const int & viewport_y, const int & width, const int & height)
{
	GLubyte *pixel_data;
	FILE *fp;
	BitmapHeader header;
	BitmapInfoHeader info;
	int alignmentParam;
	int j=0;
	int x;
	int y;
	unsigned char zero=0;

	///////////////////////////////////////////////////////////////////////////
	// チェック用の処理
	// データ格納のサイズを設定
	//glPixelStorei(GL_PACK_ALIGNMENT ,2);

	// データ格納の横幅に収まる幅のバイトの倍数を取得
	glGetIntegerv( GL_PACK_ALIGNMENT,&alignmentParam);

	// 1画素３バイトとするとひとつの行は3*widthバイト＋アルファ
	int glByteWidth;	// 実際の横幅のバイト数

	// データの幅のバイト数がalignmentParamの倍数であるかをチェック
	if( width*3%alignmentParam == 0)
		glByteWidth = width*3;
	else
		// そうでなければ，alignmentParamの倍数にあわせた幅のバイトサイズにする
		glByteWidth = width*3 + alignmentParam - (width*3)%alignmentParam;

	///////////////////////////////////////////////////////////////////////////
	// 読み込み
	// メモリ確保
	pixel_data = (GLubyte*)malloc((glByteWidth)*(height)*(sizeof(GLubyte)));
	// OpenGLによるピクセルデータの読み出し
//	glReadBuffer(GL_FRONT);
	glReadBuffer(GL_BACK);
	glReadPixels(
		viewport_x,viewport_y,
		width,height,
	    GL_RGB,
		GL_UNSIGNED_BYTE,
		pixel_data);

	///////////////////////////////////////////////////////////////////////////
	// ファイルオープン
	if( ( fp = fopen(filename, "wb") )==NULL){
		return -1;
	}
	//ヘッダ構造体の初期化
	InitHeaders(&header, &info);
	//Bitmapサイズ
	info.width = width;
	info.height = height;
	int writeWidth;

	///////////////////////////////////////////////////////////////////////////
	// BITMAP用の横幅の処理
	// データの幅のバイト数が4の倍数であるかをチェック
	if( width*3%4 == 0)
		writeWidth = width*3;
	else
		// そうでなければ，4の倍数にあわせた幅のバイトサイズにする
		writeWidth = width*3 + 4 - (width*3)%4;

	///////////////////////////////////////////////////////////////////////////
	//ファイル容量
	header.filesize =
		writeWidth*height		//ビット情報量
		+ 14					//BitmapHeader構造体サイズ
		+ 40;					//BitmapInfoHeader構造体サイズ
	//ヘッダ書き込み
	WriteHeader(&header,fp);
	WriteInfoHeader(&info,fp);

	///////////////////////////////////////////////////////////////////////////
	// イメージデータ書き込み
	for( y=0 ; y < height ; y++ ){
		// データをBGRの順で書き込み
		for( x=0 ; x < width ; x++ ){
			j=fwrite((pixel_data+x*3+glByteWidth*y+2),sizeof(GLubyte),1,fp);
			j=fwrite((pixel_data+x*3+glByteWidth*y+1),sizeof(GLubyte),1,fp);
			j=fwrite((pixel_data+x*3+glByteWidth*y),sizeof(GLubyte),1,fp);
		}
		// 幅のバイト数が4の倍数でないときは０で埋める
		if( width*3%4 != 0)
			for( int j=0;j<4-(width*3)%4;j++)
				fwrite(&zero,sizeof(GLubyte),1,fp);
	}

	///////////////////////////////////////////////////////////////////////////
	// メモリ開放
	free(pixel_data);
	// ファイルクローズ
	fclose(fp);
	return 0;
}
