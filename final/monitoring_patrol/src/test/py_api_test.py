#!/usr/bin/env python
# -*- coding: utf-8 -*-

import requests #pip install requests

# # GET���\�b�h�ɂ��Web�y�[�W�̎擾
# url = 'https://atmarkit.itmedia.co.jp/ait/articles/2209/20/news032.html'
# res = requests.get(url)
# # URL�p�����[�^�[
# url = 'https://httpbin.org/get'
# pyaload = {"key1":"10", "key2":"string"}
# res = requests.get(url, params=payload)
# print(res.url)  # https://httpbin.org/get?key1=10&key2=string
# # GET���\�b�h�ɂ��Web�y�[�W�̎擾
# res = requests.get(url, params=payload)
# # �X�e�[�^�X�R�[�h
# print(res.status_code)
# # �G���R�[�f�B���O
# print(res.encoding)  # ISO-8859-1
# print(res.apparent_encoding)  # SHIFT_JIS
# # ���X�|���X�w�b�_�[
# print(res.headers)
# # ���X�|���X�w�b�_�[�̒l�͑啶������������ʂ��Ȃ������`��
# print(res.headers['Content-Type'])  # example--> text/html
# print(res.headers['content-type'])
# # �J�X�^���w�b�_�[
# headers = {'user-agent': 'Mozilla/5.0'}
# res = requests.get(url, headers=headers)


# r = requests.post(url)
# r = requests.put(url)
# r = requests.delete(url)
# print("raw response: " + r.raw)
# print("text response: " + r.text)

# PC�ʒu���擾
# present_location = [0.0,0.0]
# geo_request_url = 'https://get.geojs.io/v1/ip/geo.json'
# data = requests.get(geo_request_url).json()
# present_location[0] = data['latitude']
# present_location[1] = data['longitude']
# print("Geojs latitude:" + data['latitude'])
# print("Geojs longitude:" + data['longitude'])
# print("Geojs country:" + data['country'])
# print(data['region'])
# print(data['city'])


nearest_aed_url = 'https://aed.azure-mobile.net/api/AEDSearch?lat=35.713&lng=139.760&r=300'
nearest_aed_data = requests.get(nearest_aed_url).json()
#print(nearest_aed_data)
# print(nearest_aed_data['DIST'])
# NEAREST_AED_DIST = 0
#n1 = len(nearest_aed_data)

dict1 = nearest_aed_data[0]


print()

#str_n1 = str(n1)
#print('\n nearest_aed_dist =' + str_n1)
