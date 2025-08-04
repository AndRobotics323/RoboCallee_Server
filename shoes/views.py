from django.http import HttpResponse
from django.shortcuts import render, get_object_or_404, redirect
from django.utils import timezone
# from server_to_fms.srv import TryOnRequest


from robocallee_fms.srv import ShoeRequest 

import rclpy
from rclpy.node import Node


from .models import Product
# from .forms import QuestionForm, AnswerForm


from django.http import HttpResponseNotAllowed

from django.core.paginator import Paginator  
from django.contrib.auth.decorators import login_required

from django.db.models import Q


# Create your views here.

service_name = 'request_service'

global_counter = 0



def index(request):
    global global_counter


    request.session.set_expiry(3600)  # 초 단위
 

    if 'customer_id' not in request.session:
        global_counter += 1
        request.session['customer_id'] = global_counter


    print(request.session['customer_id'] )

    shoes_list = Product.objects.all()


    kw = request.GET.get('kw', '')  # 검색어

    # print(kw)
    if kw:
        shoes_list = shoes_list.filter(
        Q(name__icontains=kw)   # 제목 검색
    ).distinct()

    # print(shoes_list)

    # shoes_list = Product.objects.order_by('-create_date')[:10]

    context = {'shoes_list': shoes_list, 'kw': kw}

    return render(request,  'shoes/shoes_list.html'  , context )
    #return HttpResponse("Hello, world! This is the Pybo index page.")



def detail(request, shoe_id):
    shoe = get_object_or_404(Product, pk=shoe_id)
    context = {'shoe': shoe}
    return render(request, 'shoes/shoes_detail.html', context)







# rclpy 관련

rclpy.init(args=None)

node = rclpy.create_node('try_on_client')
# client = node.create_client(TryOnRequest, 'try_on_service')
client = node.create_client(ShoeRequest, service_name )

while not client.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('Service not available, waiting again...')

# if not client.wait_for_service(timeout_sec=3.0):
#     node.destroy_node()
#     rclpy.shutdown()
#     print("Service not available")



def try_on(request):

    if request.method == 'POST':
        shoe_id = request.POST.get('shoe_id')
        shoe = get_object_or_404(Product, id=shoe_id)

        if shoe.stock > 0:
            shoe.stock -= 1
            shoe.save()
            # message = f"{shoe.name} 구매 완료! 남은 재고: {shoe.stock}"
        else:
            message = f"{shoe.name} 품절입니다."

        customer_id = request.session.get('customer_id')


            
        # req = TryOnRequest.Request()
        # req.customer_id = str(customer_id)
        # req.shoe_name = shoe.name


        req = ShoeRequest.Request()

        req.requester = "customer" 
        req.model = shoe.model
        req.size = shoe.size
        req.color = shoe.color
        # req.x = shoe.x
        # req.y = shoe.y

        req.x = int(shoe.x)
        req.y = int(shoe.y) #여기 나중에 바뀌어야
        req.customer_id = int(customer_id)

# string requester
# string model
# int32 size
# string color
# int32 x
# int32 y
# int32 customer_id

        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            response = future.result()

            
            estimated_mins = -1
            if response.accepted == True:
                estimated_mins = 0
            
            # estimated_mins = response.estimated_mins



            # node.destroy_node()
            # rclpy.shutdown()
            # return response.success, response.message
            return render(request, 'shoes/try_on.html', {'customer_id': customer_id, 'estimated_mins':estimated_mins} )

        else:
            # node.destroy_node()
            # rclpy.shutdown()
            print("Service call failed" )


    return redirect('index')  # GET으로 접근한 경우





    # context = {'shoe': shoe}



    # return render(request, 'shoes/try_on.html', context)