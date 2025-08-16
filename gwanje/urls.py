from django.urls import path

# from gwanje import views
from . import views

app_name = 'gwanje' 

urlpatterns = [
    path('cam', views.gwanje_cam , name='gwanje_cam' ), 

    path('video_feed/', views.video_feed, name='video_feed'),
    # path('markers/', views.markers_api, name='markers_api'),

    path('ocr_from_flask_stream/', views.ocr_from_flask_stream, name='ocr_from_flask_stream'),
    path('april_from_flask_stream/', views.april_from_flask_stream, name='april_from_flask_stream'),



    path('come_here/', views.come_here, name='come_here'),
    path('done_employee/', views.done_employee, name='done_employee'),




]




